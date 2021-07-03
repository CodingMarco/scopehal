/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL v0.1                                                                                                      *
*                                                                                                                      *
* Copyright (c) 2012-2020 Andrew D. Zonenberg, Mike Walters                                                            *
* All rights reserved.                                                                                                 *
*                                                                                                                      *
* Redistribution and use in source and binary forms, with or without modification, are permitted provided that the     *
* following conditions are met:                                                                                        *
*                                                                                                                      *
*    * Redistributions of source code must retain the above copyright notice, this list of conditions, and the         *
*      following disclaimer.                                                                                           *
*                                                                                                                      *
*    * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the       *
*      following disclaimer in the documentation and/or other materials provided with the distribution.                *
*                                                                                                                      *
*    * Neither the name of the author nor the names of any contributors may be used to endorse or promote products     *
*      derived from this software without specific prior written permission.                                           *
*                                                                                                                      *
* THIS SOFTWARE IS PROVIDED BY THE AUTHORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED   *
* TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL *
* THE AUTHORS BE HELD LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES        *
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR       *
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE       *
* POSSIBILITY OF SUCH DAMAGE.                                                                                          *
*                                                                                                                      *
***********************************************************************************************************************/

#include "scopehal.h"
#include "HPOscilloscope.h"
#include "EdgeTrigger.h"
#include <random>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Construction / destruction

HPOscilloscope::HPOscilloscope(SCPITransport* transport)
	: SCPIOscilloscope(transport)
	, m_triggerArmed(false)
	, m_triggerOneShot(false)
{
	// Be sure to use "." as decimal separator
	setlocale(LC_ALL, "C");

	//Second last digit of the model number is the number of channels
	std::string model_number = m_model;
	model_number.erase(
		std::remove_if(
			model_number.begin(),
			model_number.end(),
			[]( char const& c ) -> bool { return !std::isdigit(c); }
		),
		model_number.end()
	);
	int nchans = 1;//(std::stoi(model_number) / 10) % 10;
	is545X2model = std::stoi(model_number) % 10 == 2;

	m_transport->SendCommand(":ACQ:POINTS 512");
	m_transport->SendCommand(":DISP:SCREEN OFF");
	m_transport->SendCommand(":DISPLAY:CONNECT OFF");
	//m_transport->SendCommand(":TIM:REF CENTER");
	//m_transport->SendCommand(":TIM:RANGE 1");
	//m_transport->SendCommand(":tim:delay 1.5E-6");

	for(int i=0; i<nchans; i++)
	{
		//Hardware name of the channel
		std::string chname = std::string("CHAN") + std::to_string(i+1);

		//Color the channels based on Keysight's standard color sequence (yellow-green-violet-pink)
		std::string color = "#ffffff";
		switch(i)
		{
			case 0:
				color = "#ffff00";
				break;

			case 1:
				color = "#32ff00";
				break;

			case 2:
				color = "#5578ff";
				break;

			case 3:
				color = "#ff0084";
				break;
		}

		//Create the channel
		auto chan = new OscilloscopeChannel(
				this,
				chname,
				OscilloscopeChannel::CHANNEL_TYPE_ANALOG,
				color,
				1,
				i,
				true);
		m_channels.push_back(chan);
		chan->SetDefaultDisplayName();

		//Configure transport format to raw 8-bit int
		m_transport->SendCommand(":WAV:SOUR " + chname);
		m_transport->SendCommand(":WAV:FORM COMP");

		//Request all points when we download
		m_transport->SendCommand(":ACQ:TYPE NORM");
	}
	m_analogChannelCount = nchans;

	//Add the external trigger input
	m_extTrigChannel = new OscilloscopeChannel(
		this,
		"EX",
		OscilloscopeChannel::CHANNEL_TYPE_TRIGGER,
		"",
		1,
		m_channels.size(),
		true);
	m_channels.push_back(m_extTrigChannel);
}

HPOscilloscope::~HPOscilloscope()
{
	m_transport->SendCommand(":DISP:SCREEN ON");
	m_transport->SendCommand(":RUN");
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Accessors

std::string HPOscilloscope::GetDriverNameInternal()
{
		return "hp54500";
}

unsigned int HPOscilloscope::GetInstrumentTypes()
{
	return Instrument::INST_OSCILLOSCOPE;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Device interface functions

void HPOscilloscope::FlushConfigCache()
{
	std::lock_guard<std::recursive_mutex> lock(m_cacheMutex);

	m_channelOffsets.clear();
	m_channelVoltageRanges.clear();
	m_channelCouplings.clear();
	m_channelsEnabled.clear();

	delete m_trigger;
	m_trigger = NULL;
}

bool HPOscilloscope::IsChannelEnabled(size_t i)
{
	//ext trigger should never be displayed
	if(i == m_extTrigChannel->GetIndex())
		return false;

	if(i >= m_analogChannelCount)
		return false;

	{
		std::lock_guard<std::recursive_mutex> lock(m_cacheMutex);
		if(m_channelsEnabled.find(i) != m_channelsEnabled.end())
			return m_channelsEnabled[i];
	}

	std::string reply;
	{
		std::lock_guard<std::recursive_mutex> lock(m_mutex);
		m_transport->SendCommand(m_channels[i]->GetColonHwname() + ":DISP?");
		reply = m_transport->ReadReply();
	}

	std::lock_guard<std::recursive_mutex> lock(m_cacheMutex);
	if(reply == "0")
	{
		m_channelsEnabled[i] = false;
		return false;
	}
	else
	{
		m_channelsEnabled[i] = true;
		return true;
	}
}

void HPOscilloscope::EnableChannel(size_t i)
{
	{
		std::lock_guard<std::recursive_mutex> lock(m_mutex);
		m_transport->SendCommand(m_channels[i]->GetColonHwname() + ":DISP 1");
	}

	std::lock_guard<std::recursive_mutex> lock2(m_cacheMutex);
	m_channelsEnabled[i] = true;
}

void HPOscilloscope::DisableChannel(size_t i)
{
	{
		std::lock_guard<std::recursive_mutex> lock(m_mutex);
		m_transport->SendCommand(m_channels[i]->GetColonHwname() + ":DISP 0");
	}


	std::lock_guard<std::recursive_mutex> lock2(m_cacheMutex);
	m_channelsEnabled[i] = false;
}

OscilloscopeChannel::CouplingType HPOscilloscope::GetChannelCoupling(size_t i)
{
	{
		std::lock_guard<std::recursive_mutex> lock(m_cacheMutex);
		if(m_channelCouplings.find(i) != m_channelCouplings.end())
			return m_channelCouplings[i];
	}

		std::string coup_reply;
	{
		std::lock_guard<std::recursive_mutex> lock(m_mutex);
		m_transport->SendCommand(m_channels[i]->GetColonHwname() + ":COUP?");
		coup_reply = m_transport->ReadReply();
	}

	OscilloscopeChannel::CouplingType coupling;
	if(coup_reply == "DC")
		coupling = OscilloscopeChannel::COUPLE_DC_1M;
	else if(coup_reply == "DCF")
		coupling = OscilloscopeChannel::COUPLE_DC_50;
	else
		coupling = OscilloscopeChannel::COUPLE_AC_1M;

	std::lock_guard<std::recursive_mutex> lock(m_cacheMutex);
	m_channelCouplings[i] = coupling;
	return coupling;
}

void HPOscilloscope::SetChannelCoupling(size_t i, OscilloscopeChannel::CouplingType type)
{
	{
		std::lock_guard<std::recursive_mutex> lock(m_mutex);
		switch(type)
		{
			case OscilloscopeChannel::COUPLE_DC_50:
					m_transport->SendCommand(m_channels[i]->GetColonHwname() + ":COUP DCF");
				break;

			case OscilloscopeChannel::COUPLE_AC_1M:
				m_transport->SendCommand(m_channels[i]->GetColonHwname() + ":COUP AC");
				break;

			case OscilloscopeChannel::COUPLE_DC_1M:
				m_transport->SendCommand(m_channels[i]->GetColonHwname() + ":COUP DC");
				break;

			default:
				LogError("Invalid coupling for channel\n");
		}
	}

	std::lock_guard<std::recursive_mutex> lock(m_cacheMutex);
	m_channelCouplings[i] = type;
}

double HPOscilloscope::GetChannelAttenuation(size_t i)
{
	{
		std::lock_guard<std::recursive_mutex> lock(m_cacheMutex);
		if(m_channelAttenuations.find(i) != m_channelAttenuations.end())
			return m_channelAttenuations[i];
	}

	std::string reply;
	{
		std::lock_guard<std::recursive_mutex> lock(m_mutex);
		m_transport->SendCommand(m_channels[i]->GetColonHwname() + ":PROB?");
		reply = m_transport->ReadReply();
	}

	double atten = stod(reply);
	std::lock_guard<std::recursive_mutex> lock(m_cacheMutex);
	m_channelAttenuations[i] = atten;
	return atten;
}

void HPOscilloscope::SetChannelAttenuation(size_t /*i*/, double /*atten*/)
{
	//FIXME
}

int HPOscilloscope::GetChannelBandwidthLimit(size_t i)
{
	{
		std::lock_guard<std::recursive_mutex> lock(m_cacheMutex);
		if(m_channelBandwidthLimits.find(i) != m_channelBandwidthLimits.end())
			return m_channelBandwidthLimits[i];
	}


	std::string reply;
	{
		std::lock_guard<std::recursive_mutex> lock(m_mutex);
		m_transport->SendCommand(m_channels[i]->GetColonHwname() + ":HFR?");
		reply = m_transport->ReadReply();
	}

	int bwl;
	if(reply == "1")
			bwl = 20;
	else
		bwl = 0;

	std::lock_guard<std::recursive_mutex> lock(m_cacheMutex);
	m_channelBandwidthLimits[i] = bwl;
	return bwl;
}

void HPOscilloscope::SetChannelBandwidthLimit(size_t /*i*/, unsigned int /*limit_mhz*/)
{
	//FIXME
}

double HPOscilloscope::GetChannelVoltageRange(size_t i)
{
	{
		std::lock_guard<std::recursive_mutex> lock(m_cacheMutex);
		if(m_channelVoltageRanges.find(i) != m_channelVoltageRanges.end())
			return m_channelVoltageRanges[i];
	}

	std::string reply;

	{
		std::lock_guard<std::recursive_mutex> lock(m_mutex);
		m_transport->SendCommand(m_channels[i]->GetColonHwname() + ":RANGE?");

		reply = m_transport->ReadReply();
	}

	double range = stod(reply);
	std::lock_guard<std::recursive_mutex> lock(m_cacheMutex);
	m_channelVoltageRanges[i] = range;
	return range;
}

void HPOscilloscope::SetChannelVoltageRange(size_t i, double range)
{
	{
		std::lock_guard<std::recursive_mutex> lock(m_cacheMutex);
		m_channelVoltageRanges[i] = range;
	}

	std::lock_guard<std::recursive_mutex> lock(m_mutex);
	char cmd[128];
	snprintf(cmd, sizeof(cmd), "%s:RANGE %.4f", m_channels[i]->GetColonHwname().c_str(), range);
	//std::string cmd = m_channels[i]->GetColonHwname() + ":RANGE " + std::to_string(range);
	m_transport->SendCommand(cmd);
}

OscilloscopeChannel* HPOscilloscope::GetExternalTrigger()
{
	//FIXME
	return NULL;
}

double HPOscilloscope::GetChannelOffset(size_t i)
{
	{
		std::lock_guard<std::recursive_mutex> lock(m_cacheMutex);

		if(m_channelOffsets.find(i) != m_channelOffsets.end())
			return m_channelOffsets[i];
	}

	std::string reply;
	{
		std::lock_guard<std::recursive_mutex> lock(m_mutex);
		m_transport->SendCommand(m_channels[i]->GetColonHwname() + ":OFFS?");
		reply = m_transport->ReadReply();
	}

	double offset = stod(reply);
	offset = -offset;

	std::lock_guard<std::recursive_mutex> lock(m_cacheMutex);
	m_channelOffsets[i] = offset;
	return offset;
}

void HPOscilloscope::SetChannelOffset(size_t i, double offset)
{
	{
		std::lock_guard<std::recursive_mutex> lock(m_cacheMutex);
		m_channelOffsets[i] = offset;
	}

	std::lock_guard<std::recursive_mutex> lock(m_mutex);
	char cmd[128];
	snprintf(cmd, sizeof(cmd), "%s:OFFS %.4f", m_channels[i]->GetColonHwname().c_str(), -offset);
	m_transport->SendCommand(cmd);
}

Oscilloscope::TriggerMode HPOscilloscope::PollTrigger()
{
	return TRIGGER_MODE_TRIGGERED;
	if (!m_triggerArmed)
		return TRIGGER_MODE_STOP;

	std::lock_guard<std::recursive_mutex> lock(m_mutex);
	m_transport->SendCommand(":TIM:MODE?");
	std::string trigger_mode = m_transport->ReadReply();

	if(trigger_mode == "AUTO")
		return TRIGGER_MODE_RUN;
	else if(trigger_mode == "TRIG")
	{
		m_triggerArmed = false;
		return TRIGGER_MODE_TRIGGERED;
	}
	else // "SINGLE"
	{
		return TRIGGER_MODE_STOP;
	}
}

bool HPOscilloscope::AcquireData()
{
	//LogDebug("Acquiring data\n");

	std::lock_guard<std::recursive_mutex> lock(m_mutex);
	LogIndenter li;

	digitizeActiveChannels();

	std::map<int, std::vector<AnalogWaveform*> > pending_waveforms;
	for(size_t i=0; i<m_analogChannelCount; i++)
	{
		if(!IsChannelEnabled(i))
			continue;

		// Set source & get preamble
		m_transport->SendCommand(":WAV:SOUR " + m_channels[i]->GetHwname());

		WaveformPreamble waveformParameters = GetWaveformPreamble();

		//Figure out the sample rate
		int64_t ps_per_sample = round(waveformParameters.xincrement * 1e12f);
		//LogDebug("%ld ps/sample\n", ps_per_sample);

		//LogDebug("length = %d\n", length);

		//Set up the capture we're going to store our data into
		//(no TDC data available on Agilent scopes?)
		AnalogWaveform* cap = new AnalogWaveform;
		cap->m_timescale = ps_per_sample;
		cap->m_triggerPhase = 0;
		cap->m_startTimestamp = time(NULL);
		double t = GetTime();
		cap->m_startPicoseconds = (t - floor(t)) * 1e12f;

		//Ask for the data
		m_transport->SendCommand(":WAV:DATA?");

		//Read the length header
		char tmp[16] = {0};
		m_transport->ReadRawData(2, (unsigned char*)tmp);
		int num_digits = atoi(tmp+1);
		//LogDebug("num_digits = %d\n", num_digits);
		m_transport->ReadRawData(num_digits, (unsigned char*)tmp);
		int actual_len = atoi(tmp);
		//LogDebug("actual_len = %d\n", actual_len);

		uint8_t* temp_buf = new uint8_t[actual_len / sizeof(uint8_t)];



		//Read the actual data
		m_transport->ReadRawData(actual_len, (unsigned char*)temp_buf);
		//Discard trailing newline
		m_transport->ReadRawData(1, (unsigned char*)tmp);

		const double lsb = m_channels[i]->GetVoltageRange() / pow(2, 8);
		std::uniform_real_distribution<double> rand(-lsb*2, lsb*2);
		std::default_random_engine re;
		re.seed(clock());
		//Format the capture
		cap->Resize(waveformParameters.length);
		bool do_rand = false;
		double arand = 0;
		for(size_t j=0; j<waveformParameters.length; j++)
		{
			if(do_rand)
			{
				arand = rand(re);
			}
			cap->m_offsets[j] = j;
			cap->m_durations[j] = 1;
			cap->m_samples[j] = waveformParameters.yincrement
					* (temp_buf[j] - waveformParameters.yreference)
					+ waveformParameters.yorigin + arand;
		}

		//Done, update the data
		pending_waveforms[i].push_back(cap);

		//Clean up
		delete[] temp_buf;
	}

	//Now that we have all of the pending waveforms, save them in sets across all channels
	m_pendingWaveformsMutex.lock();
	size_t num_pending = 1;	//TODO: segmented capture mode
	for(size_t i=0; i<num_pending; i++)
	{
		SequenceSet s;
		for(size_t j=0; j<m_analogChannelCount; j++)
		{
			if(IsChannelEnabled(j))
				s[m_channels[j]] = pending_waveforms[j][i];
		}
		m_pendingWaveforms.push_back(s);
	}
	m_pendingWaveformsMutex.unlock();

	//TODO: support digital channels

	//Re-arm the trigger if not in one-shot mode
	if(!m_triggerOneShot)
	{
		//m_transport->SendCommand(":DIG");
		m_triggerArmed = true;
	}

	//LogDebug("Acquisition done\n");
	return true;
}

void HPOscilloscope::Start()
{
	std::lock_guard<std::recursive_mutex> lock(m_mutex);
	//m_transport->SendCommand(":DIG");
	m_triggerArmed = true;
	m_triggerOneShot = false;
}

void HPOscilloscope::StartSingleTrigger()
{
	std::lock_guard<std::recursive_mutex> lock(m_mutex);
	//m_transport->SendCommand(":DIG");
	m_triggerArmed = true;
	m_triggerOneShot = true;
}

void HPOscilloscope::Stop()
{
	std::lock_guard<std::recursive_mutex> lock(m_mutex);
	m_transport->SendCommand(":STOP");
	m_triggerArmed = false;
	m_triggerOneShot = true;
}

bool HPOscilloscope::IsTriggerArmed()
{
	return m_triggerArmed;
}

std::vector<uint64_t> HPOscilloscope::GetSampleRatesNonInterleaved()
{
	const int64_t k = 1000;
	const int64_t m = k*k;
	const int64_t g = m*k;

	// TODO: Implement sample rate limit on 545X0 models

//	std::vector<uint64_t> ret = { 10, 25, 50, 100, 250, 500, 1*k, int(2.5*k), 5*k, 10*k, 25*k, 50*k,
//								  100*k, 250*k, 500*k, 1*m, int(2.5*m), 5*m, 10*m, 25*m, 50*m, 100*m,
//								  250*m, 500*m, 1*g, 2*g };

	std::vector<uint64_t> ret = { int(2.5*k), 5*k, 10*k, 25*k, 50*k,
								  100*k, 250*k, 500*k, 1*m, int(2.5*m), 5*m, 10*m, 25*m, 50*m, 100*m,
								  250*m, 500*m, 1*g, 2*g };

	return ret;
}

std::vector<uint64_t> HPOscilloscope::GetSampleRatesInterleaved()
{
	return GetSampleRatesNonInterleaved();
}

std::set<Oscilloscope::InterleaveConflict> HPOscilloscope::GetInterleaveConflicts()
{
	//FIXME
	std::set<Oscilloscope::InterleaveConflict> ret;
	return ret;
}

std::vector<uint64_t> HPOscilloscope::GetSampleDepthsNonInterleaved()
{
		return { 512, 1024, 2048, 4096, 8192, 16384, 32768 };
}

std::vector<uint64_t> HPOscilloscope::GetSampleDepthsInterleaved()
{
	//FIXME
	std::vector<uint64_t> ret;
	return ret;
}

uint64_t HPOscilloscope::GetSampleRate()
{
	std::lock_guard<std::recursive_mutex> lock(m_mutex);
	m_transport->SendCommand(":TIM:SAMP:CLOCK?");

	return readNR3<uint64_t>();
}

uint64_t HPOscilloscope::GetSampleDepth()
{
	std::lock_guard<std::recursive_mutex> lock(m_mutex);
	m_transport->SendCommand(":ACQ:POINTS?");

	return readNR3<uint64_t>();
}

void HPOscilloscope::SetSampleDepth(uint64_t depth)
{
	std::lock_guard<std::recursive_mutex> lock(m_mutex);
	LogDebug("%s", ("Setting sample depth to " + std::to_string(depth) + "\n").c_str());
	// Since the 54500 scopes need sample depths which are exactly powers of 2,
	// they are parsed wrong from the TimebasePropertiesDialog.
	// 512 stays 512, 1024 becomes 1, 2048 becomes 2, and so on. We correct that here.
	if(depth != 512)
		depth *= 1024;
	depth = 512;
	m_transport->SendCommand(std::string(":ACQ:POINTS ") + std::to_string(depth));
}

void HPOscilloscope::SetSampleRate(uint64_t rate)
{
	if(rate >= GetSampleRatesNonInterleaved()[0])
	{
		std::lock_guard<std::recursive_mutex> lock(m_mutex);
		LogDebug("%s", ("Setting sample rate to " + std::to_string(rate) + "\n").c_str());
		m_transport->SendCommand(std::string(":TIM:SAMP:CLOCK ") + std::to_string(rate));
	}
}

void HPOscilloscope::SetTriggerOffset(int64_t /*offset*/)
{
	//FIXME
}

int64_t HPOscilloscope::GetTriggerOffset()
{
	//FIXME
	return 0;
}

bool HPOscilloscope::IsInterleaving()
{
	return false;
}

bool HPOscilloscope::SetInterleaving(bool /*combine*/)
{
	return false;
}

void HPOscilloscope::PullTrigger()
{
	std::lock_guard<std::recursive_mutex> lock(m_mutex);

	//Figure out what kind of trigger is active.
	m_transport->SendCommand(":TRIG:MODE?");
	std::string reply = m_transport->ReadReply();
	if (reply == "EDGE")
		PullEdgeTrigger();

	//Unrecognized trigger type
	else
	{
		LogWarning("Unknown trigger type \"%s\"\n", reply.c_str());
		m_trigger = NULL;
		return;
	}
}

/**
	@brief Reads settings for an edge trigger from the instrument
 */
void HPOscilloscope::PullEdgeTrigger()
{
	//Clear out any triggers of the wrong type
	if( (m_trigger != NULL) && (dynamic_cast<EdgeTrigger*>(m_trigger) != NULL) )
	{
		delete m_trigger;
		m_trigger = NULL;
	}

	//Create a new trigger if necessary
	if(m_trigger == NULL)
		m_trigger = new EdgeTrigger(this);
	EdgeTrigger* et = dynamic_cast<EdgeTrigger*>(m_trigger);

	std::lock_guard<std::recursive_mutex> lock(m_mutex);

	//Source
	m_transport->SendCommand(":TRIG:SOUR?");
	std::string reply = m_transport->ReadReply();
	auto chan = GetChannelByHwName(reply);
	et->SetInput(0, StreamDescriptor(chan, 0), true);
	if(!chan)
		LogWarning("Unknown trigger source %s\n", reply.c_str());

	//Level
	m_transport->SendCommand(":TRIG:LEV?");
	reply = m_transport->ReadReply();
	et->SetLevel(stof(reply));

	//Edge slope
	m_transport->SendCommand(":TRIG:SLOPE?");
	reply = m_transport->ReadReply();
	if (reply == "POS")
		et->SetType(EdgeTrigger::EDGE_RISING);
	else if (reply == "NEG")
		et->SetType(EdgeTrigger::EDGE_FALLING);
}

void HPOscilloscope::PushTrigger()
{
	auto et = dynamic_cast<EdgeTrigger*>(m_trigger);
	if(et)
		PushEdgeTrigger(et);

	else
		LogWarning("Unknown trigger type (not an edge)\n");
}

/**
	@brief Pushes settings for an edge trigger to the instrument
 */
void HPOscilloscope::PushEdgeTrigger(EdgeTrigger* trig)
{
	std::lock_guard<std::recursive_mutex> lock(m_mutex);

	//Source
	m_transport->SendCommand(std::string(":TRIG:SOURCE ") + trig->GetInput(0).m_channel->GetHwname());

	//Level
	char tmp[32];
	snprintf(tmp, sizeof(tmp), ":TRIG:LEV %.3f", trig->GetLevel());
	m_transport->SendCommand(tmp);

	//Slope
	switch((int)trig->GetType())
	{
		case EdgeTrigger::EDGE_RISING:
			m_transport->SendCommand(":TRIG:SLOPE POS");
			break;
		case EdgeTrigger::EDGE_FALLING:
			m_transport->SendCommand(":TRIG:SLOPE NEG");
			break;

		default:
			return;
	}
}

WaveformPreamble HPOscilloscope::GetWaveformPreamble()
{
	std::lock_guard<std::recursive_mutex> lock(m_mutex);

	m_transport->SendCommand(":WAV:PRE?");
	std::string reply = m_transport->ReadReply();

	std::stringstream ss(reply);
	WaveformPreamble ret;

	ss >> ret.format; ss.ignore();
	ss >> ret.type; ss.ignore();
	ss >> ret.length; ss.ignore();
	ss >> ret.average_count; ss.ignore();
	ss >> ret.xincrement; ss.ignore();
	ss >> ret.xorigin; ss.ignore();
	ss >> ret.xreference; ss.ignore();
	ss >> ret.yincrement; ss.ignore();
	ss >> ret.yorigin; ss.ignore();
	ss >> ret.yreference;

	return ret;
}

void HPOscilloscope::digitizeActiveChannels()
{
	std::lock_guard<std::recursive_mutex> lock(m_mutex);
	std::string digitizeCommand = ":DIG ";
	for(size_t i = 0; i < m_analogChannelCount; i++)
	{
		if(m_channels[i]->IsEnabled())
			digitizeCommand.append(m_channels[i]->GetHwname() + ",");
	}
	digitizeCommand.pop_back();
	m_transport->SendCommand(digitizeCommand);
}

std::vector<std::string> HPOscilloscope::GetTriggerTypes()
{
	std::vector<std::string> ret;
	ret.push_back(EdgeTrigger::GetTriggerName());
	return ret;
}


template<typename T>
T HPOscilloscope::HPOscilloscope::readNR3()
{
	std::string reply = m_transport->ReadReply();
	T replyNumber;
	std::stringstream ss(reply);
	ss >> replyNumber;

	return replyNumber;
}
