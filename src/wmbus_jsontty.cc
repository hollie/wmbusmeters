/*
 Copyright (C) 2026 Lieven Hollevoet (gpl-3.0-or-later)

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include"wmbus.h"
#include"wmbus_common_implementation.h"
#include"wmbus_utils.h"
#include"serial.h"

#include<assert.h>
#include<pthread.h>
#include<semaphore.h>
#include<errno.h>
#include<unistd.h>

using namespace std;

struct WMBusJsonTTY : public virtual BusDeviceCommonImplementation
{
    bool ping();
    string getDeviceId();
    string getDeviceUniqueId();
    LinkModeSet getLinkModes();
    void deviceReset();
    bool deviceSetLinkModes(LinkModeSet lms);
    LinkModeSet supportedLinkModes() { return Any_bit; }
    int numConcurrentLinkModes() { return 0; }
    bool canSetLinkModes(LinkModeSet desired_modes) { return true; }

    void processSerialData();
    void simulate() { }

    WMBusJsonTTY(string bus_alias, shared_ptr<SerialDevice> serial,
                 shared_ptr<SerialCommunicationManager> manager);
    ~WMBusJsonTTY() { }

private:

    string extractRawField(const string &json_line);
    
    vector<uchar> read_buffer_;
    LinkModeSet link_modes_;
    vector<uchar> received_payload_;
};

shared_ptr<BusDevice> openJsonTTY(Detected detected,
                                  shared_ptr<SerialCommunicationManager> manager,
                                  shared_ptr<SerialDevice> serial_override)
{
    string bus_alias = detected.specified_device.bus_alias;
    string device = detected.found_file;
    int bps = detected.found_bps;
    
    // Default to 115200 if not specified
    if (bps == 0) bps = 115200;

    if (detected.specified_device.command != "")
    {
        string identifier = "cmd_" + to_string(detected.specified_device.index);

        vector<string> args;
        vector<string> envs;
        args.push_back("-c");
        args.push_back(detected.specified_device.command);

        auto serial = manager->createSerialDeviceCommand(identifier, "/bin/sh", args, envs, "jsontty");
        WMBusJsonTTY *imp = new WMBusJsonTTY(bus_alias, serial, manager);
        return shared_ptr<BusDevice>(imp);
    }

    if (serial_override)
    {
        WMBusJsonTTY *imp = new WMBusJsonTTY(bus_alias, serial_override, manager);
        imp->markAsNoLongerSerial();
        return shared_ptr<BusDevice>(imp);
    }
    
    auto serial = manager->createSerialDeviceTTY(device.c_str(), bps, PARITY::NONE, "jsontty");
    WMBusJsonTTY *imp = new WMBusJsonTTY(bus_alias, serial, manager);
    return shared_ptr<BusDevice>(imp);
}

WMBusJsonTTY::WMBusJsonTTY(string bus_alias, shared_ptr<SerialDevice> serial,
                           shared_ptr<SerialCommunicationManager> manager) :
    BusDeviceCommonImplementation(bus_alias, DEVICE_JSONTTY, manager, serial, true)
{
    debug("(jsontty) constructor called, resetting device\n");
    reset();
    debug("(jsontty) device reset complete, ready to receive data\n");
}

bool WMBusJsonTTY::ping()
{
    return true;
}

string WMBusJsonTTY::getDeviceId()
{
    return "?";
}

string WMBusJsonTTY::getDeviceUniqueId()
{
    return "?";
}

LinkModeSet WMBusJsonTTY::getLinkModes() {
    return link_modes_;
}

void WMBusJsonTTY::deviceReset()
{
    debug("(jsontty) deviceReset called\n");
    if (serial())
    {
        debug("(jsontty) serial device fd=%d opened=%d working=%d\n", 
              serial()->fd(), serial()->opened(), serial()->working());
    }
}

bool WMBusJsonTTY::deviceSetLinkModes(LinkModeSet lms)
{
    return true;
}

string WMBusJsonTTY::extractRawField(const string &json_line)
{
    // Simple JSON parser to extract "raw" field value
    // Looks for: "raw":"<hex_value>" or "raw": "<hex_value>"
    
    size_t raw_pos = json_line.find("\"raw\"");
    if (raw_pos == string::npos)
    {
        // Try without quotes around key (less common but possible)
        raw_pos = json_line.find("raw");
        if (raw_pos == string::npos) return "";
    }
    
    // Find the colon after "raw"
    size_t colon_pos = json_line.find(":", raw_pos);
    if (colon_pos == string::npos) return "";
    
    // Skip whitespace after colon
    size_t start_pos = colon_pos + 1;
    while (start_pos < json_line.length() && 
           (json_line[start_pos] == ' ' || json_line[start_pos] == '\t'))
    {
        start_pos++;
    }
    
    // Expect opening quote
    if (start_pos >= json_line.length() || json_line[start_pos] != '"')
    {
        return "";
    }
    start_pos++; // Skip opening quote
    
    // Find closing quote
    size_t end_pos = json_line.find("\"", start_pos);
    if (end_pos == string::npos) return "";
    
    // Extract the hex string
    string raw_hex = json_line.substr(start_pos, end_pos - start_pos);
    
    debug("(jsontty) extracted raw field: %s\n", raw_hex.c_str());
    
    return raw_hex;
}

void WMBusJsonTTY::processSerialData()
{
    vector<uchar> data;

    // Receive and accumulate serial data until a full line has been received.
    serial()->receive(&data);

    if (data.size() > 0)
    {
        debug("(jsontty) received %zu bytes from serial\n", data.size());
    }

    read_buffer_.insert(read_buffer_.end(), data.begin(), data.end());

    // Process complete JSON lines (terminated by newline)
    for (;;)
    {
        // Look for newline character
        auto newline_pos = find(read_buffer_.begin(), read_buffer_.end(), '\n');
        if (newline_pos == read_buffer_.end())
        {
            // No complete line yet
            break;
        }
        
        // Extract the complete line (excluding the newline)
        string json_line(read_buffer_.begin(), newline_pos);
        
        // Remove the processed line from buffer (including newline)
        read_buffer_.erase(read_buffer_.begin(), newline_pos + 1);
        
        // Skip empty lines
        if (json_line.empty() || json_line.find_first_not_of(" \t\r") == string::npos)
        {
            continue;
        }
        
        debug("(jsontty) received JSON line: %s\n", json_line.c_str());
        
        // Extract the raw hex field from JSON
        string raw_hex = extractRawField(json_line);
        if (raw_hex.empty())
        {
            verbose("(jsontty) no 'raw' field found in JSON line\n");
            continue;
        }
        
        // Convert hex string to binary
        vector<uchar> payload;
        bool ok = hex2bin(raw_hex, &payload);
        if (!ok)
        {
            warning("(jsontty) failed to parse hex data: %s\n", raw_hex.c_str());
            continue;
        }
        
        if (payload.empty())
        {
            verbose("(jsontty) empty payload after hex conversion\n");
            continue;
        }
        
        debug("(jsontty) converted %zu hex chars to %zu bytes\n", raw_hex.length(), payload.size());
        
        // Check if this looks like a valid wmbus frame
        if (payload.size() < 10)
        {
            warning("(jsontty) payload too short (%zu bytes), ignoring\n", payload.size());
            continue;
        }
        
        // Remove the CRCs from the payload since the receiver includes them
        // but wmbusmeters expects them to be already removed
        debug("(jsontty) removing CRCs from payload\n");
        removeAnyDLLCRCs(payload);
        debug("(jsontty) payload after CRC removal: %zu bytes\n", payload.size());
        
        if (payload.size() == 0)
        {
            warning("(jsontty) empty payload after CRC removal\n");
            continue;
        }
        
        // Pass the telegram to the handler
        // The payload should now have CRCs removed as expected by wmbusmeters
        AboutTelegram about("", 0, LinkMode::UNKNOWN, FrameType::WMBUS);
        handleTelegram(about, payload);
    }
}

AccessCheck detectJsonTTY(Detected *detected, shared_ptr<SerialCommunicationManager> manager)
{
    // JsonTTY devices cannot be auto-detected, they must be manually configured
    // This function should not be called due to detectSKIP in the device list
    return AccessCheck::NoSuchDevice;
}
