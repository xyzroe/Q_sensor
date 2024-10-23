const {Zcl} = require('zigbee-herdsman');

const fz = require('zigbee-herdsman-converters/converters/fromZigbee');
const tz = require('zigbee-herdsman-converters/converters/toZigbee');
const exposes = require('zigbee-herdsman-converters/lib/exposes');
const reporting = require('zigbee-herdsman-converters/lib/reporting');
const ota = require('zigbee-herdsman-converters/lib/ota');
const utils = require('zigbee-herdsman-converters/lib/utils');
const globalStore = require('zigbee-herdsman-converters/lib/store');
const e = exposes.presets;
const ea = exposes.access;

const {
    deviceEndpoints,
    iasZoneAlarm,
    temperature,
    humidity,
    pressure,
    co2,
    illuminance,
    numeric,
    identify,
    enumLookup,
} = require('zigbee-herdsman-converters/lib/modernExtend');

const defaultReporting = {min: 30, max: 300, change: 0};

const definition = {
    zigbeeModel: ['Q_sensor'],
    model: 'Q_sensor',
    vendor: 'xyzroe',
    description: 'Multi-functional Zigbee Air Quality Sensor',
    ota: ota.zigbeeOTA,
    extend: [
        deviceEndpoints({endpoints: {1: 1, 2: 2, 3: 3, 4: 4, 5: 5, 6: 6, 7: 7, 8: 8, 9: 9, 10: 10, 11: 11, 12: 12, 13: 13}}),
        iasZoneAlarm({zoneType: 'occupancy', zoneAttributes: ['alarm_1', 'tamper']}),
        temperature({endpointNames: ['1', '2', '3', '4']}),
        humidity({endpointNames: ['2', '4']}),
        pressure({endpointNames: ['3']}),
        co2({endpointNames: ['4']}),
        illuminance({endpointNames: ['5']}),
        identify(),
        numeric({
            name: 'voc',
            unit: 'points',
            cluster: 'genAnalogInput',
            attribute: 'presentValue',
            description: 'VOC index',
            access: 'STATE_GET',
            endpointNames: ['6'],
            reporting: defaultReporting,
        }),
        numeric({
            name: 'adc1',
            unit: 'mV',
            cluster: 'genAnalogInput',
            attribute: 'presentValue',
            description: 'ADC1 value',
            access: 'STATE_GET',
            endpointNames: ['7'],
            reporting: defaultReporting,
        }),
        numeric({
            name: 'adc2',
            unit: 'mV',
            cluster: 'genAnalogInput',
            attribute: 'presentValue',
            description: 'ADC2 value',
            access: 'STATE_GET',
            endpointNames: ['8'],
            reporting: defaultReporting,
        }),
        enumLookup({
            name: 'possition',
            lookup: {Incorrect: 0, Horizontal: 1, Vertical: 2},
            cluster: 'genMultistateValue',
            attribute: 'presentValue',
            description: 'Device position',
            access: 'STATE_GET',
            endpointNames: ['10'],
            reporting: defaultReporting,
        }),
        numeric({
            name: 'pitch',
            unit: '°',
            cluster: 'genAnalogInput',
            attribute: 'presentValue',
            description: 'Pitch value',
            access: 'STATE_GET',
            endpointNames: ['11'],
            reporting: defaultReporting,
            precision: 6,
        }),
        numeric({
            name: 'roll',
            unit: '°',
            cluster: 'genAnalogInput',
            attribute: 'presentValue',
            description: 'Roll value',
            access: 'STATE_GET',
            endpointNames: ['12'],
            reporting: defaultReporting,
            precision: 6,
        }),
        numeric({
            name: 'yaw',
            unit: '°',
            cluster: 'genAnalogInput',
            attribute: 'presentValue',
            description: 'Yaw value',
            access: 'STATE_GET',
            endpointNames: ['13'],
            reporting: defaultReporting,
            precision: 6,
        }),
    ],
    meta: {multiEndpoint: true},
};

module.exports = definition;
