-- loading lua-yarp binding library
require("yarp")

-- Lua 5.2 lacks a global 'unpack' function
local unpack = table.unpack or unpack

-- Globals
local NUMBER_OF_PREVIOUS_ITERATIONS = 5
local FILTER_FACTOR = 5

local SensorDataProcessor = {}

--
-- Performs a deep copy of provided table (only table children, no metadata).
-- From http://lua-users.org/wiki/CopyTable
--
-- @param target source Table
--
-- @return cloned Table
--
function cloneTable(target)
    local target_type = type(target)
    local copy
    if target_type == 'table' then
        copy = {}
        for target_key, target_value in next, target, nil do
            copy[cloneTable(target_key)] = cloneTable(target_value)
        end
    else -- number, string, boolean, etc
        copy = target
    end
    return copy
end

--
-- Filters odd values (peaks) from current data set.
-- Current implementation: discard data if current and previous iterations
-- exceed initial value by a constant factor.
--
-- @param reference Table of sensor data for previous iteration
--
function filterPeaks(references, currentData)
    local hasPeak = function(ref, tgt)
        return tgt > ref * FILTER_FACTOR
    end

    local refIter = references[1]

    for i, part in ipairs(currentData) do
        for j, decValue in ipairs(part) do
            local ref = refIter[i][j]

            if hasPeak(ref, decValue) then
                local recurrentPeak = true

                for k = 2, #references do
                    recurrentPeak = recurrentPeak and hasPeak(ref, references[k][i][j])
                end

                if not recurrentPeak then
                    part[j] = ref
                end
            end
        end
    end
end

--
-- Creates a SensorDataProcessor instance.
--
-- @param processor Table of input parameters
--
-- @return SensorDataProcessor
--
SensorDataProcessor.new = function(self, processor)
    local obj = {
        accumulator = "",
        dataReady = false,
        currentSensorData = nil,
        processor = processor,
        stamp = 0,
        previousIterations = {}
    }

    setmetatable(obj, self)
    self.__index = self
    return obj
end

--
-- Consumes fetched data from sensors.
--
-- @param str String of new data to be appended to the accumulator
--
SensorDataProcessor.accept = function(self, str)
    str = str or ""
    self.accumulator = self.accumulator .. str
    self:tryConsume()
end

--
-- Calls further processing methods if conditions are satisfied,
-- sets flag if data is ready to be forwarded to receiver.
--
SensorDataProcessor.tryConsume = function(self)
    if self.processor.evaluateCondition(self.accumulator) then
        self.currentSensorData, self.accumulator = self.processor.process(self.accumulator)

        if self.currentSensorData then
            -- TODO: users may want to disable or use other filter
            if #self.previousIterations == NUMBER_OF_PREVIOUS_ITERATIONS then
                filterPeaks(self.previousIterations, self.currentSensorData)
                table.remove(self.previousIterations, 1)
            end

            self.dataReady = true
            self.stamp = self.stamp + 1

            table.insert(self.previousIterations, cloneTable(self.currentSensorData))
        else
            print("message dropped")
        end
    end
end

--
-- Get current stamp (incremented after data is ready on each iteration).
--
-- @return Number
--
SensorDataProcessor.getStamp = function(self)
    return self.stamp
end

--
-- Factory function, creates a mean SensorDataProcessor.
--
-- @param properties Table of input parameters
-- @param properties.parts String identifier for main data streams (arm, elbow, hand)
-- @param properties.subparts String identifier for secondary data streams (X, Y, Z)
-- @param properties.hexValues Number of hexadecimal values contained in a single data stream
-- @param properties.hexSize Number of characters a hexValue is comprised of
--
-- @return Table of output parameters
-- @return Table.evaluateCondition Function, see docs below
-- @return Table.process Function, see docs below
--
local createMeanProcessor = function(properties)
    -- identifier for invalid input values
    local invalidValue = tonumber(string.rep("F", properties.hexSize), 16)
    --local separators = {"\r\n", "\n", "\r"}
    --local storage = {}

    --
    -- Splits input into lines.
    --
    -- @param str String of raw data, contains newline characters.
    --
    -- @return Table
    --
    local extractMessages = function(str)
        local lines = {}

        for line in string.gmatch(str, "%S+") do
            table.insert(lines, line)
        end

        return lines
    end

    --
    -- Groups (by part/subpart) and sorts input data table.
    --
    -- @param lines Table of full message data (parts + subparts)
    --
    local preprocessMessages = function(lines)
        local temp = {}
        local i = 1
        local nparts, nsubparts = #properties.parts, #properties.subparts

        if #lines < nparts * (nsubparts + 1) then return end

        -- always assume the following order:
        --  part {I, J, K}
        --  subpart X
        --  subpart Y
        --  subpart Z
        while i <= #lines do
            local ttemp = {} -- part + 3 subparts
            table.insert(ttemp, lines[i])
            i = i + 1

            for j = 1, nsubparts do
                table.insert(ttemp, lines[i])
                i = i + 1
            end

            table.insert(temp, ttemp)
        end

        -- sort alphabetically by first element, i.e. the part ("I", "J", "K")
        table.sort(temp, function(a, b)
            return a[1] < b[1]
        end)

        -- clear original table
        for i, v in ipairs(lines) do lines[i] = nil end

        -- fill with correct sequence of parts and subparts, one level of depth
        -- remove duplicate parts: https://stackoverflow.com/a/12397571
        i = 1
        while i <= #temp do
            if i ~= 1 and temp[i][1] == temp[i - 1][1] then
                table.remove(temp, i)
            else
                for j, group in ipairs(temp[i]) do
                    table.insert(lines, group)
                end
                i = i + 1
            end
        end
    end

    --
    -- Tokenizes input string into elements of constant width.
    --
    -- @param str input String
    -- @param n Number of characters each token is comprised of
    --
    -- @return Table of split tokens
    --
    local splitString = function(str, n)
        n = math.floor(n or 0)
        if n <= 0 then return {""} end
        local t = {}

        for i = 1, math.floor(str:len() / n) do
            local start = 1 + n * (i - 1)
            table.insert(t, str:sub(start, start + n - 1))
        end

        return t
    end

    --
    -- Calculates arithmetic mean of input data.
    --
    -- @param varargs of input Numbers
    --
    -- @return Number as arithmetic mean of input data
    --
    local calculateMean = function(...)
        if select('#', ...) == 0 then return 0 end
        local sum = 0

        for i, value in ipairs({...}) do
            sum = sum + value
        end

        return math.floor(sum / select('#', ...))
    end

    --
    -- Extracts sensor value from sanitized streams and applies selected algorithm.
    --
    -- @param Table of preprocessed message Strings.
    --
    -- @return Table on success, nil on failure
    --
    local doWork = function(lines)
        local nline = 0
        local storage = {}

        for i, part in ipairs(properties.parts) do
            nline = nline + 1
            local line = lines[nline]
            if not line or line ~= part then return nil end
            local partArray = {}

            for j, subpart in ipairs(properties.subparts) do
                nline = nline + 1
                line = lines[nline]
                if not line or line:sub(1, 1) ~= subpart then return nil end
                local substring = line:sub(2)
                if substring:len() ~= properties.hexValues * properties.hexSize then return nil end
                local thex = splitString(substring, properties.hexSize)
                if #thex ~= properties.hexValues then return nil end

                for k, hexString in ipairs(thex) do
                    local dec = tonumber(hexString, 16)
                    if not dec then return nil end
                    if dec == invalidValue then dec = 0 end
                    local tuple = partArray[k] or {}
                    table.insert(tuple, dec)

                    if not partArray[k] then
                        table.insert(partArray, tuple)
                    end
                end
            end

            for k, tuple in ipairs(partArray) do
                partArray[k] = calculateMean(unpack(tuple))
            end

            table.insert(storage, partArray)
        end

        return storage
    end

    --
    -- Tests whether provided string represents a complete stream of data (all parts and subparts).
    --
    -- @param str input String
    --
    -- @return Boolean
    --
    local evaluateCondition = function(str)
        local temp = {}
        local occurrences = 0
        local singleOccurrenceParts = {}

        for i, id in ipairs(properties.parts) do
            local j = 1

            -- find all occurrences of part 'id'
            while j <= str:len() do
                local n
                n, j = str:find(id, j, true)

                -- no more occurrences, exit loop
                if not n then break end

                -- store index in associative array for given part
                local t = temp[id] or {}
                table.insert(t, n)
                temp[id] = t

                j = j + 1
            end

            -- a part identifier is missing
            if not temp[id] then return false end

            -- if this part has a single occurrence, store its id
            if #temp[id] == 1 then
                table.insert(singleOccurrenceParts, id)
            end

            occurrences = occurrences + #temp[id]
        end

        -- must have at least nº of parts + 1
        if not (occurrences > #properties.parts) then return false end

        local lastOccurrencePos = 0

        -- find position of last occurrence
        for id, ocs in pairs(temp) do
            for j, n in ipairs(ocs) do
                if n > lastOccurrencePos then lastOccurrencePos = n end
            end
        end

        -- make sure that a part with a single occurrence is not last
        for i, id in ipairs(singleOccurrenceParts) do
            if temp[id][1] == lastOccurrencePos then return false end
        end

        return true
    end

    --
    -- Extract the biggest chunk of text containing full part frames.
    --
    -- @param str input string
    --
    -- @return desired String chunk
    -- @return rightmost remainder String chunk
    --
    local extractSubstring = function(str)
        local first, last = str:len(), str:len()

        for i, part in ipairs(properties.parts) do
            local f = str:find(part, 1, true)
            if f < first then first = f end
            local l = str:reverse():find(part, 1, true)
            if l < last then last = l end
        end

        last = str:len() - last
        return str:sub(first, last), str:sub(last + 1)
    end

    --
    -- Main execution block, processes raw data.
    --
    -- @param str String of raw data accumulated by the processor in previous iterations
    --
    -- @return Table of parsed messages
    -- @return String of unprocessed data left by this iteration
    --
    local process = function(str)
        local substring, remainder = extractSubstring(str)
        local lines = extractMessages(substring)
        preprocessMessages(lines)
        return doWork(lines), remainder
    end

    return {
        evaluateCondition = evaluateCondition,
        process = process
    }
end

local sensorDataProcessor

--
-- Method called when the port monitor is created.
--
-- @param options
--
-- @return Boolean
--
PortMonitor.create = function(options)
    print("INITIALIZING AMOR SENSORS")
    -- TODO: read options from .ini file
    local meanProcessor = createMeanProcessor{
        parts = {"I", "J", "K"},
        subparts = {"X", "Y", "Z"},
        hexValues = 16,
        hexSize = 3
    }
    sensorDataProcessor = SensorDataProcessor:new(meanProcessor)
    return true;
end

--
-- Method called when port monitor is destroyed.
--
PortMonitor.destroy = function()
    print("DESTROYING PORT MONITOR")
    sensorDataProcessor = nil
end

--
-- Method called when the port receives new data.
-- If false is returned, the data will be ignored
-- and update() will never be called.
--
-- @param thing The Things abstract data type
--
-- @return Boolean
--
PortMonitor.accept = function(thing)
    bt = thing:asBottle()

    if bt == nil then
        print("bot_modifier.lua: got wrong data type (expected type Bottle)")
        return false
    end

    sensorDataProcessor:accept(bt:get(0):asString())
    return sensorDataProcessor.dataReady
end

--
-- Method called to forward processed data.
--
-- @param thing The Things abstract data type
--
-- @return Things
--
PortMonitor.update = function(thing)
    print(string.format("in update [%d]", sensorDataProcessor:getStamp()))
    sensorDataProcessor.dataReady = false
    local vec = yarp.Vector()

    for i, part in ipairs(sensorDataProcessor.currentSensorData) do
        for j, decValue in ipairs(part) do
            vec:push_back(decValue)
        end
    end

    thing:setPortWriter(vec)
    return thing
end
