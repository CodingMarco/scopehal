/***********************************************************************************************************************
*                                                                                                                      *
* libscopehal v0.1                                                                                                     *
*                                                                                                                      *
* Copyright (c) 2012-2023 Andrew D. Zonenberg and contributors                                                         *
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
#include "HP54500Oscilloscope.h"
#include "EdgeTrigger.h"
#include "PulseWidthTrigger.h"
#include "NthEdgeBurstTrigger.h"

using namespace std;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Construction / destruction

HP54500Oscilloscope::HP54500Oscilloscope(SCPITransport* transport)
	: SCPIDevice(transport)
	, SCPIInstrument(transport)
	, m_triggerArmed(false)
	, m_triggerOneShot(false)
{
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
    int nchans = (std::stoi(model_number) / 10) % 10;
    is545X2model = std::stoi(model_number) % 10 == 2;

	for(int i=0; i<nchans; i++)
	{
		//Hardware name of the channel
		string chname = string("CHAN1");
		chname[4] += i;

		//Color the channels based on Agilent's standard color sequence (yellow-green-violet-pink)
		string color = "#ffffff";
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
			color,
			Unit(Unit::UNIT_FS),
			Unit(Unit::UNIT_VOLTS),
			Stream::STREAM_TYPE_ANALOG,
			i);
		m_channels.push_back(chan);
		chan->SetDefaultDisplayName();
	}
	m_analogChannelCount = nchans;
}

HP54500Oscilloscope::~HP54500Oscilloscope()
{
}

void HP54500Oscilloscope::ConfigureWaveform(string channel)
{
	//Select the channel to apply settings to
	//NOTE: this also enables the channel
	m_transport->SendCommand(":WAV:SOUR " + channel);

	//Configure transport format to raw 8-bit int
	m_transport->SendCommand(":WAV:FORM BYTE");

	//Request all points when we download
	m_transport->SendCommand(":WAV:POIN:MODE RAW");
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Accessors

string HP54500Oscilloscope::GetDriverNameInternal()
{
	return "agilent";
}

unsigned int HP54500Oscilloscope::GetInstrumentTypes()
{
	return Instrument::INST_OSCILLOSCOPE;
}

uint32_t HP54500Oscilloscope::GetInstrumentTypesForChannel(size_t /*i*/)
{
	return Instrument::INST_OSCILLOSCOPE;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Device interface functions

void HP54500Oscilloscope::FlushConfigCache()
{
	lock_guard<recursive_mutex> lock(m_cacheMutex);

	m_channelOffsets.clear();
	m_channelVoltageRanges.clear();
	m_channelCouplings.clear();
	m_channelAttenuations.clear();
	m_channelBandwidthLimits.clear();
	m_channelsEnabled.clear();
	m_probeTypes.clear();

	m_sampleRateValid = false;
	m_sampleDepthValid = false;

	delete m_trigger;
	m_trigger = NULL;
}

bool HP54500Oscilloscope::IsAnalogChannel(size_t i)
{
	return GetOscilloscopeChannel(i)->GetType(0) == Stream::STREAM_TYPE_ANALOG;
}

bool HP54500Oscilloscope::IsChannelEnabled(size_t i)
{
	//ext trigger should never be displayed
	if(i == m_extTrigChannel->GetIndex())
		return false;

	{
		lock_guard<recursive_mutex> lock(m_cacheMutex);
		if(m_channelsEnabled.find(i) != m_channelsEnabled.end())
			return m_channelsEnabled[i];
	}

	string reply;
	{
		lock_guard<recursive_mutex> lock(m_mutex);
		m_transport->SendCommand(GetOscilloscopeChannel(i)->GetHwname() + ":DISP?");
		reply = m_transport->ReadReply();
	}

	lock_guard<recursive_mutex> lock(m_cacheMutex);
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

void HP54500Oscilloscope::EnableChannel(size_t i)
{
	{
		lock_guard<recursive_mutex> lock(m_mutex);
		m_transport->SendCommand(GetOscilloscopeChannel(i)->GetHwname() + ":DISP ON");
	}

    ConfigureWaveform(GetOscilloscopeChannel(i)->GetHwname());

	lock_guard<recursive_mutex> lock2(m_cacheMutex);
	m_channelsEnabled[i] = true;
}

void HP54500Oscilloscope::DisableChannel(size_t i)
{
	{
		lock_guard<recursive_mutex> lock(m_mutex);
		m_transport->SendCommand(GetOscilloscopeChannel(i)->GetHwname() + ":DISP OFF");
	}


	lock_guard<recursive_mutex> lock2(m_cacheMutex);
	m_channelsEnabled[i] = false;
}

vector<OscilloscopeChannel::CouplingType> HP54500Oscilloscope::GetAvailableCouplings(size_t /*i*/)
{
	vector<OscilloscopeChannel::CouplingType> ret;
	ret.push_back(OscilloscopeChannel::COUPLE_DC_1M);
	ret.push_back(OscilloscopeChannel::COUPLE_AC_1M);
	ret.push_back(OscilloscopeChannel::COUPLE_DC_50);
	ret.push_back(OscilloscopeChannel::COUPLE_GND);
	return ret;
}

OscilloscopeChannel::CouplingType HP54500Oscilloscope::GetChannelCoupling(size_t i)
{
	if(!IsAnalogChannel(i))
		return OscilloscopeChannel::COUPLE_SYNTHETIC;

	{
		lock_guard<recursive_mutex> lock(m_cacheMutex);
		if(m_channelCouplings.find(i) != m_channelCouplings.end())
			return m_channelCouplings[i];
	}

	string coup_reply, imp_reply;
	{
		lock_guard<recursive_mutex> lock(m_mutex);
		m_transport->SendCommand(GetOscilloscopeChannel(i)->GetHwname() + ":COUP?");
		coup_reply = m_transport->ReadReply();
		m_transport->SendCommand(GetOscilloscopeChannel(i)->GetHwname() + ":IMP?");
		imp_reply = m_transport->ReadReply();
	}

	OscilloscopeChannel::CouplingType coupling;
	if(coup_reply == "AC")
		coupling = OscilloscopeChannel::COUPLE_AC_1M;
	else /*if(coup_reply == "DC")*/
	{
		if(imp_reply == "ONEM")
			coupling = OscilloscopeChannel::COUPLE_DC_1M;
		else /*if(imp_reply == "FIFT")*/
			coupling = OscilloscopeChannel::COUPLE_DC_50;
	}

	lock_guard<recursive_mutex> lock(m_cacheMutex);
	m_channelCouplings[i] = coupling;
	return coupling;
}

void HP54500Oscilloscope::SetChannelCoupling(size_t i, OscilloscopeChannel::CouplingType type)
{
	// If there's a smart probe on this channel, the coupling is fixed to 50ohm so bail out.
	GetProbeType(i);
	if (m_probeTypes[i] == SmartProbe)
		return;

	{
		lock_guard<recursive_mutex> lock(m_mutex);
		switch(type)
		{
			case OscilloscopeChannel::COUPLE_DC_50:
				m_transport->SendCommand(GetOscilloscopeChannel(i)->GetHwname() + ":COUP DC");
				m_transport->SendCommand(GetOscilloscopeChannel(i)->GetHwname() + ":IMP FIFT");
				break;

			case OscilloscopeChannel::COUPLE_AC_1M:
				m_transport->SendCommand(GetOscilloscopeChannel(i)->GetHwname() + ":IMP ONEM");
				m_transport->SendCommand(GetOscilloscopeChannel(i)->GetHwname() + ":COUP AC");
				break;

			case OscilloscopeChannel::COUPLE_DC_1M:
				m_transport->SendCommand(GetOscilloscopeChannel(i)->GetHwname() + ":IMP ONEM");
				m_transport->SendCommand(GetOscilloscopeChannel(i)->GetHwname() + ":COUP DC");
				break;

			default:
				LogError("Invalid coupling for channel\n");
		}
	}

	lock_guard<recursive_mutex> lock(m_cacheMutex);
	m_channelCouplings[i] = type;
}

double HP54500Oscilloscope::GetChannelAttenuation(size_t i)
{
	{
		lock_guard<recursive_mutex> lock(m_cacheMutex);
		if(m_channelAttenuations.find(i) != m_channelAttenuations.end())
			return m_channelAttenuations[i];
	}

	string reply;
	{
		lock_guard<recursive_mutex> lock(m_mutex);
		m_transport->SendCommand(GetOscilloscopeChannel(i)->GetHwname() + ":PROB?");
		reply = m_transport->ReadReply();
	}

	double atten = stod(reply);
	lock_guard<recursive_mutex> lock(m_cacheMutex);
	m_channelAttenuations[i] = atten;
	return atten;
}

void HP54500Oscilloscope::SetChannelAttenuation(size_t i, double atten)
{
	// If there's a SmartProbe or AutoProbe on this channel, the attenuation is fixed so bail out.
	GetProbeType(i);
	if (m_probeTypes[i] != None)
		return;

	{
		lock_guard<recursive_mutex> lock(m_mutex);
		PushFloat(GetOscilloscopeChannel(i)->GetHwname() + ":PROB", atten);
	}

	lock_guard<recursive_mutex> lock(m_cacheMutex);
	m_channelAttenuations[i] = atten;
}

unsigned int HP54500Oscilloscope::GetChannelBandwidthLimit(size_t i)
{
	{
		lock_guard<recursive_mutex> lock(m_cacheMutex);
		if(m_channelBandwidthLimits.find(i) != m_channelBandwidthLimits.end())
			return m_channelBandwidthLimits[i];
	}


	string reply;
	{
		lock_guard<recursive_mutex> lock(m_mutex);
		m_transport->SendCommand(GetOscilloscopeChannel(i)->GetHwname() + ":BWL?");
		reply = m_transport->ReadReply();
	}

	unsigned int bwl;
	if(reply == "1")
		bwl = 25;
	else
		bwl = 0;

	lock_guard<recursive_mutex> lock(m_cacheMutex);
	m_channelBandwidthLimits[i] = bwl;
	return bwl;
}

void HP54500Oscilloscope::SetChannelBandwidthLimit(size_t /*i*/, unsigned int /*limit_mhz*/)
{
	//FIXME
}

float HP54500Oscilloscope::GetChannelVoltageRange(size_t i, size_t /*stream*/)
{
	if(GetOscilloscopeChannel(i)->GetType(0) != Stream::STREAM_TYPE_ANALOG)
		return 1;

	{
		lock_guard<recursive_mutex> lock(m_cacheMutex);
		if(m_channelVoltageRanges.find(i) != m_channelVoltageRanges.end())
			return m_channelVoltageRanges[i];
	}

	string reply;

	{
		lock_guard<recursive_mutex> lock(m_mutex);
		m_transport->SendCommand(GetOscilloscopeChannel(i)->GetHwname() + ":RANGE?");

		reply = m_transport->ReadReply();
	}

	float range = stof(reply);
	lock_guard<recursive_mutex> lock(m_cacheMutex);
	m_channelVoltageRanges[i] = range;
	return range;
}

void HP54500Oscilloscope::SetChannelVoltageRange(size_t i, size_t /*stream*/, float range)
{
	{
		lock_guard<recursive_mutex> lock(m_cacheMutex);
		m_channelVoltageRanges[i] = range;
	}

	lock_guard<recursive_mutex> lock(m_mutex);
	char cmd[128];
	snprintf(cmd, sizeof(cmd), "%s:RANGE %.4f", GetOscilloscopeChannel(i)->GetHwname().c_str(), range);
	m_transport->SendCommand(cmd);
}

OscilloscopeChannel* HP54500Oscilloscope::GetExternalTrigger()
{
	//FIXME
	return NULL;
}

float HP54500Oscilloscope::GetChannelOffset(size_t i, size_t /*stream*/)
{
	if(!IsAnalogChannel(i))
		return 0;

	{
		lock_guard<recursive_mutex> lock(m_cacheMutex);

		if(m_channelOffsets.find(i) != m_channelOffsets.end())
			return m_channelOffsets[i];
	}

	string reply;
	{
		lock_guard<recursive_mutex> lock(m_mutex);
		m_transport->SendCommand(GetOscilloscopeChannel(i)->GetHwname() + ":OFFS?");
		reply = m_transport->ReadReply();
	}

	float offset = stof(reply);
	offset = -offset;

	lock_guard<recursive_mutex> lock(m_cacheMutex);
	m_channelOffsets[i] = offset;
	return offset;
}

void HP54500Oscilloscope::SetChannelOffset(size_t i, size_t /*stream*/, float offset)
{
	{
		lock_guard<recursive_mutex> lock(m_cacheMutex);
		m_channelOffsets[i] = offset;
	}

	lock_guard<recursive_mutex> lock(m_mutex);
	char cmd[128];
	snprintf(cmd, sizeof(cmd), "%s:OFFS %.4f", GetOscilloscopeChannel(i)->GetHwname().c_str(), -offset);
	m_transport->SendCommand(cmd);
}

Oscilloscope::TriggerMode HP54500Oscilloscope::PollTrigger()
{
	if (!m_triggerArmed)
		return TRIGGER_MODE_STOP;

	// Based on example from 6000 Series Programmer's Guide
	// Section 10 'Synchronizing Acquisitions' -> 'Polling Synchronization With Timeout'
	lock_guard<recursive_mutex> lock(m_mutex);
	m_transport->SendCommand(":OPER:COND?");
	string ter = m_transport->ReadReply();
	int cond = atoi(ter.c_str());

	// Check bit 3 ('Run' bit)
	if((cond & (1 << 3)) != 0)
		return TRIGGER_MODE_RUN;
	else
	{
		m_triggerArmed = false;
		return TRIGGER_MODE_TRIGGERED;
	}
}

vector<uint8_t> HP54500Oscilloscope::GetWaveformData(string channel)
{
	lock_guard<recursive_mutex> lock(m_mutex);
	m_transport->SendCommand(":WAV:SOUR " + channel);
	m_transport->SendCommand(":WAV:DATA?");

	// Read the length header size
	char tmp[16] = {0};
	m_transport->ReadRawData(2, (unsigned char*)tmp);
	auto header_len = atoi(tmp+1);

	// Read data length
	m_transport->ReadRawData(header_len, (unsigned char*)tmp);
	auto data_len = atoi(tmp);

	// Read the actual data
	auto buf = vector<uint8_t>(data_len);
	m_transport->ReadRawData(data_len, &buf[0]);

	// Discard trailing newline
	m_transport->ReadRawData(1, (unsigned char*)tmp);

	return buf;
}

HP54500Oscilloscope::WaveformPreamble HP54500Oscilloscope::GetWaveformPreamble(string channel)
{
	WaveformPreamble ret;

	lock_guard<recursive_mutex> lock(m_mutex);
	m_transport->SendCommand(":WAV:SOUR " + channel);
	m_transport->SendCommand(":WAV:PRE?");
	string reply = m_transport->ReadReply();
	sscanf(reply.c_str(), "%u,%u,%zu,%u,%lf,%lf,%lf,%lf,%lf,%lf",
			&ret.format, &ret.type, &ret.length, &ret.average_count,
			&ret.xincrement, &ret.xorigin, &ret.xreference,
			&ret.yincrement, &ret.yorigin, &ret.yreference);

	return ret;
}

bool HP54500Oscilloscope::AcquireData()
{
	lock_guard<recursive_mutex> lock(m_mutex);
	LogIndenter li;

	map<int, vector<WaveformBase*> > pending_waveforms;
	for(size_t i=0; i<m_analogChannelCount; i++)
	{
		if(!IsChannelEnabled(i))
			continue;

		auto chname = GetOscilloscopeChannel(i)->GetHwname();
		auto preamble = GetWaveformPreamble(chname);

		//Figure out the sample rate
		int64_t fs_per_sample = round(preamble.xincrement * FS_PER_SECOND);

		//Set up the capture we're going to store our data into
		//(no TDC data available on Agilent scopes?)
		auto cap = new UniformAnalogWaveform;
		cap->m_timescale = fs_per_sample;
		cap->m_triggerPhase = 0;
		cap->m_startTimestamp = time(NULL);
		double t = GetTime();
		cap->m_startFemtoseconds = (t - floor(t)) * FS_PER_SECOND;

		// Format the capture
		auto buf = GetWaveformData(chname);
		if(preamble.length != buf.size())
			LogError("Waveform preamble length (%lu) does not match data length (%lu)", preamble.length, buf.size());
		cap->Resize(buf.size());
		float gain = preamble.yincrement;
		float offset = (gain * preamble.yreference) - preamble.yorigin;
		ConvertUnsigned8BitSamples(cap->m_samples.GetCpuPointer(), buf.data(), gain, offset, buf.size());

		//Done, update the data
		cap->MarkSamplesModifiedFromCpu();
		pending_waveforms[i].push_back(cap);
	}

	//Now that we have all of the pending waveforms, save them in sets across all channels
	m_pendingWaveformsMutex.lock();
	size_t num_pending = 1;	//TODO: segmented capture mode
	for(size_t i=0; i<num_pending; i++)
	{
		SequenceSet s;
		for (size_t j = 0; j < m_channels.size(); j++)
			if(IsChannelEnabled(j) && pending_waveforms.find(j) != pending_waveforms.end())
				s[GetOscilloscopeChannel(j)] = pending_waveforms[j][i];
		m_pendingWaveforms.push_back(s);
	}
	m_pendingWaveformsMutex.unlock();

	//Re-arm the trigger if not in one-shot mode
	if(!m_triggerOneShot)
	{
		m_transport->SendCommand(":SING");
		m_triggerArmed = true;
	}

	return true;
}

void HP54500Oscilloscope::Start()
{
	lock_guard<recursive_mutex> lock(m_mutex);
	m_transport->SendCommand("SING");
	m_triggerArmed = true;
	m_triggerOneShot = false;
}

void HP54500Oscilloscope::StartSingleTrigger()
{
	lock_guard<recursive_mutex> lock(m_mutex);
	m_transport->SendCommand("SING");
	m_triggerArmed = true;
	m_triggerOneShot = true;
}

void HP54500Oscilloscope::Stop()
{
	lock_guard<recursive_mutex> lock(m_mutex);
	m_transport->SendCommand("STOP");
	m_triggerArmed = false;
	m_triggerOneShot = true;
}

void HP54500Oscilloscope::ForceTrigger()
{
    LogError("HP54500Oscilloscope::ForceTrigger not implemented\n");
}

bool HP54500Oscilloscope::IsTriggerArmed()
{
	return m_triggerArmed;
}

static std::map<uint64_t, double> sampleRateToDuration {
	// Map sample rates to corresponding maximum on-screen time duration setting
	{8000      , 500},
	{20000     , 200},
	{40000     , 100},
	{80000     , 50},
	{200000    , 20},
	{400000    , 10},
	{800000    , 5},
	{2000000   , 2},
	{4000000   , 1},
	{8000000   , 500e-3},
	{20000000  , 200e-3},
	{40000000  , 100e-3},
	{80000000  , 50e-3},
	{200000000 , 20e-3},
	{400000000 , 10e-3},
	{500000000 , 5e-3},
	{2000000000, 2e-3},
};

vector<uint64_t> HP54500Oscilloscope::GetSampleRatesNonInterleaved()
{
	vector<uint64_t> ret;
	for (auto x: sampleRateToDuration)
		ret.push_back(x.first);

	return ret;
}

vector<uint64_t> HP54500Oscilloscope::GetSampleRatesInterleaved()
{
	//FIXME
	vector<uint64_t> ret;
	return ret;
}

set<Oscilloscope::InterleaveConflict> HP54500Oscilloscope::GetInterleaveConflicts()
{
	//FIXME
	set<Oscilloscope::InterleaveConflict> ret;
	return ret;
}

vector<uint64_t> HP54500Oscilloscope::GetSampleDepthsNonInterleaved()
{
	return {
		100,
		250,
		500,
		1000,
		2000,
		5000,
		10000,
		20000,
		50000,
		100000,
		200000,
		500000,
		1000000,
		2000000,
		4000000,
		8000000,
	};
}

vector<uint64_t> HP54500Oscilloscope::GetSampleDepthsInterleaved()
{
	//FIXME
	vector<uint64_t> ret;
	return ret;
}

uint64_t HP54500Oscilloscope::GetSampleRate()
{
	if (m_sampleRateValid)
		return m_sampleRate;

	lock_guard<recursive_mutex> lock(m_mutex);
	m_transport->SendCommand("ACQUIRE:SRATE?");
	uint64_t rate = stof(m_transport->ReadReply());
	m_sampleRate = rate;
	m_sampleRateValid = true;
	return rate;
}

uint64_t HP54500Oscilloscope::GetSampleDepth()
{
	if (m_sampleDepthValid)
		return m_sampleDepth;

	lock_guard<recursive_mutex> lock(m_mutex);
	m_transport->SendCommand("ACQUIRE:POINTS?");
	uint64_t depth = stof(m_transport->ReadReply());
	m_sampleDepth = depth;
	m_sampleDepthValid = true;
	return depth;
}

void HP54500Oscilloscope::SetSampleRateAndDepth(uint64_t rate, uint64_t depth)
{
	// Look up the maximum capture duration for the requested sample rate
	auto d = sampleRateToDuration.find(rate);
	if (d == sampleRateToDuration.end())
		return;
	auto max_duration = d->second;

	// Calculate the duration of the requested capture in seconds
	auto duration = (double)depth / (double)rate;

	// Clamp the duration to make sure we achieve at least the requested sample rate
	duration = min(duration, max_duration);

	lock_guard<recursive_mutex> lock(m_mutex);
	PushFloat("TIMEBASE:RANGE", duration);
	for (auto chan : m_channels)
	{
		auto ochan = dynamic_cast<OscilloscopeChannel*>(chan);
		if(!ochan)
			continue;
		if (ochan->GetType(0) == Stream::STREAM_TYPE_ANALOG)
		{
			m_transport->SendCommand(":WAV:SOUR " + chan->GetHwname());

			// This will downsample the capture in case we ended up with a sample rate much higher than requested
			m_transport->SendCommand(":WAV:POINTS " + to_string(depth));
		}
	}
}

void HP54500Oscilloscope::SetSampleDepth(uint64_t depth)
{
	auto rate = GetSampleRate();
	SetSampleRateAndDepth(rate, depth);
	m_sampleDepth = depth;
	m_sampleDepthValid = true;
}

void HP54500Oscilloscope::SetSampleRate(uint64_t rate)
{
	auto depth = GetSampleDepth();
	SetSampleRateAndDepth(rate, depth);
	m_sampleRate = rate;
	m_sampleRateValid = true;
}

void HP54500Oscilloscope::SetTriggerOffset(int64_t /*offset*/)
{
	//FIXME
}

int64_t HP54500Oscilloscope::GetTriggerOffset()
{
	//FIXME
	return 0;
}

bool HP54500Oscilloscope::IsInterleaving()
{
	return false;
}

bool HP54500Oscilloscope::SetInterleaving(bool /*combine*/)
{
	return false;
}

void HP54500Oscilloscope::PullTrigger()
{
	lock_guard<recursive_mutex> lock(m_mutex);

	//Figure out what kind of trigger is active.
	m_transport->SendCommand("TRIG:MODE?");
	string reply = m_transport->ReadReply();
	if (reply == "EDGE")
		PullEdgeTrigger();
	else if (reply == "GLIT")
		PullPulseWidthTrigger();
	else if (reply == "EBUR")
		PullNthEdgeBurstTrigger();

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
void HP54500Oscilloscope::PullEdgeTrigger()
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
	auto et = dynamic_cast<EdgeTrigger*>(m_trigger);

	lock_guard<recursive_mutex> lock(m_mutex);

	//Source
	m_transport->SendCommand("TRIG:SOUR?");
	string reply = m_transport->ReadReply();
	auto chan = GetOscilloscopeChannelByHwName(reply);
	et->SetInput(0, StreamDescriptor(chan, 0), true);
	if(!chan)
		LogWarning("Unknown trigger source %s\n", reply.c_str());

	//Level
	m_transport->SendCommand("TRIG:LEV?");
	reply = m_transport->ReadReply();
	et->SetLevel(stof(reply));

	//Edge slope
	m_transport->SendCommand("TRIG:SLOPE?");
	GetTriggerSlope(et, m_transport->ReadReply());
}

void HP54500Oscilloscope::PullNthEdgeBurstTrigger()
{
	//Clear out any triggers of the wrong type
	if( (m_trigger != NULL) && (dynamic_cast<NthEdgeBurstTrigger*>(m_trigger) != NULL) )
	{
		delete m_trigger;
		m_trigger = NULL;
	}

	//Create a new trigger if necessary
	if(m_trigger == NULL)
		m_trigger = new NthEdgeBurstTrigger(this);
	auto bt = dynamic_cast<NthEdgeBurstTrigger*>(m_trigger);

	lock_guard<recursive_mutex> lock(m_mutex);

	//Source
	m_transport->SendCommand("TRIG:EDGE:SOUR?");
	string reply = m_transport->ReadReply();
	auto chan = GetOscilloscopeChannelByHwName(reply);
	bt->SetInput(0, StreamDescriptor(chan, 0), true);
	if(!chan)
		LogWarning("Unknown trigger source %s\n", reply.c_str());

	//Level
	m_transport->SendCommand("TRIG:EDGE:LEV?");
	bt->SetLevel(stof(m_transport->ReadReply()));

	//Slope
	m_transport->SendCommand("TRIG:EBUR:SLOP?");
	GetTriggerSlope(bt, m_transport->ReadReply());

	//Idle time
	m_transport->SendCommand("TRIG:EBUR:IDLE?");
	bt->SetIdleTime(stof(m_transport->ReadReply()) * FS_PER_SECOND);

	//Edge number
	m_transport->SendCommand("TRIG:EBUR:COUN?");
	bt->SetEdgeNumber(stoi(m_transport->ReadReply()));
}

void HP54500Oscilloscope::PullPulseWidthTrigger()
{
	//Clear out any triggers of the wrong type
	if( (m_trigger != NULL) && (dynamic_cast<PulseWidthTrigger*>(m_trigger) != NULL) )
	{
		delete m_trigger;
		m_trigger = NULL;
	}

	//Create a new trigger if necessary
	if(m_trigger == NULL)
		m_trigger = new PulseWidthTrigger(this);
	auto pt = dynamic_cast<PulseWidthTrigger*>(m_trigger);

	lock_guard<recursive_mutex> lock(m_mutex);

	//Source
	m_transport->SendCommand("TRIG:GLIT:SOUR?");
	string reply = m_transport->ReadReply();
	auto chan = GetOscilloscopeChannelByHwName(reply);
	pt->SetInput(0, StreamDescriptor(chan, 0), true);
	if(!chan)
		LogWarning("Unknown trigger source %s\n", reply.c_str());

	//Level
	m_transport->SendCommand("TRIG:GLIT:LEV?");
	pt->SetLevel(stof(m_transport->ReadReply()));

	//Condition
	m_transport->SendCommand("TRIG:GLIT:QUAL?");
	pt->SetCondition(GetCondition(m_transport->ReadReply()));

	//Slope
	m_transport->SendCommand("TRIG:GLIT:POL?");
	GetTriggerSlope(pt, m_transport->ReadReply());

	// Bounds
	//
	// In the BETWEEN condition the bounds are stored in a different variable
	// on the scope so check & set the correct one.
	if(pt->GetCondition() == Trigger::CONDITION_BETWEEN)
	{
		m_transport->SendCommand("TRIG:GLIT:RANG?");
		reply = m_transport->ReadReply();
		stringstream ss(reply);
		string upper_bound, lower_bound;

		if (!getline(ss, upper_bound, ',') || !getline(ss, lower_bound, ','))
			LogWarning("Malformed TRIG:GLIT:RANG response: %s\n", reply.c_str());
		else
		{
			pt->SetLowerBound(stof(lower_bound) * FS_PER_SECOND);
			pt->SetUpperBound(stof(upper_bound) * FS_PER_SECOND);
		}

	}
	else
	{
		//Lower bound
		m_transport->SendCommand("TRIG:GLIT:GRE?");
		pt->SetLowerBound(stof(m_transport->ReadReply()) * FS_PER_SECOND);

		//Upper bound
		m_transport->SendCommand("TRIG:GLIT:LESS?");
		pt->SetUpperBound(stof(m_transport->ReadReply()) * FS_PER_SECOND);
	}
}

/**
	@brief Processes the slope for an edge or edge-derived trigger
 */
void HP54500Oscilloscope::GetTriggerSlope(EdgeTrigger* trig, string reply)
{
	if (reply == "POS")
		trig->SetType(EdgeTrigger::EDGE_RISING);
	else if (reply == "NEG")
		trig->SetType(EdgeTrigger::EDGE_FALLING);
	else if (reply == "EITH")
		trig->SetType(EdgeTrigger::EDGE_ANY);
	else if (reply == "ALT")
		trig->SetType(EdgeTrigger::EDGE_ALTERNATING);
	else
		LogWarning("Unknown trigger slope %s\n", reply.c_str());
}

/**
	@brief Processes the slope for an Nth edge burst trigger
 */
void HP54500Oscilloscope::GetTriggerSlope(NthEdgeBurstTrigger* trig, string reply)
{
	if (reply == "POS")
		trig->SetSlope(NthEdgeBurstTrigger::EDGE_RISING);
	else if (reply == "NEG")
		trig->SetSlope(NthEdgeBurstTrigger::EDGE_FALLING);
	else
		LogWarning("Unknown trigger slope %s\n", reply.c_str());
}

/**
	@brief Parses a trigger condition
 */
Trigger::Condition HP54500Oscilloscope::GetCondition(string reply)
{
	reply = Trim(reply);

	if(reply == "LESS")
		return Trigger::CONDITION_LESS;
	else if(reply == "GRE")
		return Trigger::CONDITION_GREATER;
	else if(reply == "RANG")
		return Trigger::CONDITION_BETWEEN;

	//unknown
	return Trigger::CONDITION_LESS;
}

void HP54500Oscilloscope::GetProbeType(size_t i)
{
	{
		lock_guard<recursive_mutex> lock(m_cacheMutex);
		if(m_probeTypes.find(i) != m_probeTypes.end())
			return;
	}

	string reply;
	{
		lock_guard<recursive_mutex> lock(m_mutex);
		m_transport->SendCommand(GetOscilloscopeChannel(i)->GetHwname() + ":PROBE:ID?");
		reply = m_transport->ReadReply();
	}

	lock_guard<recursive_mutex> lock(m_cacheMutex);
	if (reply == "AutoProbe")
		m_probeTypes[i] = AutoProbe;
	else if (reply == "NONE" || reply == "Unknown")
		m_probeTypes[i] = None;
	else
		m_probeTypes[i] = SmartProbe;
}

void HP54500Oscilloscope::PushTrigger()
{
	auto bt = dynamic_cast<NthEdgeBurstTrigger*>(m_trigger);
	auto pt = dynamic_cast<PulseWidthTrigger*>(m_trigger);
	auto et = dynamic_cast<EdgeTrigger*>(m_trigger);
	if(bt)
		PushNthEdgeBurstTrigger(bt);
	else if(pt)
		PushPulseWidthTrigger(pt);
	// Must go last
	else if(et)
		PushEdgeTrigger(et);

	else
		LogWarning("Unknown trigger type (not an edge)\n");
}

/**
	@brief Pushes settings for an edge trigger to the instrument
 */
void HP54500Oscilloscope::PushEdgeTrigger(EdgeTrigger* trig)
{
	lock_guard<recursive_mutex> lock(m_mutex);

	//Mode
	m_transport->SendCommand("TRIG:MODE EDGE");

	//Source
	m_transport->SendCommand(string("TRIG:SOURCE ") + trig->GetInput(0).m_channel->GetHwname());

	//Level
	PushFloat("TRIG:LEV", trig->GetLevel());

	//Slope
	PushSlope("TRIG:SLOPE", trig->GetType());
}

/**
	@brief Pushes settings for a Nth edge burst trigger to the instrument
 */
void HP54500Oscilloscope::PushNthEdgeBurstTrigger(NthEdgeBurstTrigger* trig)
{
	lock_guard<recursive_mutex> lock(m_mutex);

	m_transport->SendCommand("TRIG:MODE EBUR");
	m_transport->SendCommand("TRIG:EDGE:SOURCE " +
		trig->GetInput(0).m_channel->GetHwname());
	PushFloat("TRIG:EDGE:LEV", trig->GetLevel());
	PushSlope("TRIG:EBUR:SLOP", trig->GetSlope());
	PushFloat("TRIG:EBUR:IDLE", trig->GetIdleTime() * SECONDS_PER_FS);
	m_transport->SendCommand("TRIG:EBUR:COUNT " + to_string(trig->GetEdgeNumber()));
}

/**
	@brief Pushes settings for a pulse width trigger to the instrument
 */
void HP54500Oscilloscope::PushPulseWidthTrigger(PulseWidthTrigger* trig)
{
	lock_guard<recursive_mutex> lock(m_mutex);

	m_transport->SendCommand("TRIG:MODE GLIT");
	m_transport->SendCommand("TRIG:GLIT:SOURCE " +
		trig->GetInput(0).m_channel->GetHwname());
	PushSlope("TRIG:GLIT:POL", trig->GetType());
	PushCondition("TRIG:GLIT:QUAL", trig->GetCondition());
	PushFloat("TRIG:GLIT:LEV", trig->GetLevel());
	if(trig->GetCondition() == Trigger::CONDITION_BETWEEN)
	{
		m_transport->SendCommand("TRIG:GLIT:RANG " +
			to_string_sci(trig->GetUpperBound() * SECONDS_PER_FS) +
			"," +
			to_string_sci(trig->GetLowerBound() * SECONDS_PER_FS));
	}
	else
	{
		PushFloat("TRIG:GLIT:LESS", trig->GetUpperBound() * SECONDS_PER_FS);
		PushFloat("TRIG:GLIT:GRE",  trig->GetLowerBound() * SECONDS_PER_FS);
	}
}

void HP54500Oscilloscope::PushCondition(string path, Trigger::Condition cond)
{
	string cond_str;
	switch(cond)
	{
		case Trigger::CONDITION_LESS:
			cond_str = "LESS";
			break;
		case Trigger::CONDITION_GREATER:
			cond_str = "GRE";
			break;
		case Trigger::CONDITION_BETWEEN:
			cond_str = "RANG";
			break;
		default:
			return;
	}
	m_transport->SendCommand(path + " " + cond_str);
}

void HP54500Oscilloscope::PushFloat(string path, float f)
{
	m_transport->SendCommand(path + " " + to_string_sci(f));
}

void HP54500Oscilloscope::PushSlope(string path, EdgeTrigger::EdgeType slope)
{
	string slope_str;
	switch(slope)
	{
		case EdgeTrigger::EDGE_RISING:
			slope_str = "POS";
			break;
		case EdgeTrigger::EDGE_FALLING:
			slope_str = "NEG";
			break;
		case EdgeTrigger::EDGE_ANY:
			slope_str = "EITH";
			break;
		case EdgeTrigger::EDGE_ALTERNATING:
			slope_str = "ALT";
			break;
		default:
			return;
	}
	m_transport->SendCommand(path + " " + slope_str);
}

void HP54500Oscilloscope::PushSlope(string path, NthEdgeBurstTrigger::EdgeType slope)
{
	string slope_str;
	switch(slope)
	{
		case EdgeTrigger::EDGE_RISING:
			slope_str = "POS";
			break;
		case EdgeTrigger::EDGE_FALLING:
			slope_str = "NEG";
			break;
		default:
			return;
	}
	m_transport->SendCommand(path + " " + slope_str);
}

vector<string> HP54500Oscilloscope::GetTriggerTypes()
{
	vector<string> ret;
	ret.push_back(EdgeTrigger::GetTriggerName());
	ret.push_back(PulseWidthTrigger::GetTriggerName());
	ret.push_back(NthEdgeBurstTrigger::GetTriggerName());
	return ret;
}
