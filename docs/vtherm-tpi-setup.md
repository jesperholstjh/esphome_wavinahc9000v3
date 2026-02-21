# VTherm TPI Setup with Wavin AHC-9000

Use [Versatile Thermostat](https://github.com/jmcollin78/versatile_thermostat) (VTherm) in `over_switch` mode with TPI algorithm to control Wavin floor heating zones from Home Assistant.

## How It Works

VTherm's TPI algorithm calculates an on/off duty cycle based on the difference between target temperature and current room temperature. It toggles a switch entity on and off within each cycle to modulate heating power.

Instead of toggling the Wavin controller's mode (HEAT/OFF), we keep Wavin permanently in HEAT mode and control heating by changing the setpoint:

- **Switch ON** (VTherm demands heat): Set Wavin setpoint to 35 C (forces heating)
- **Switch OFF** (no demand): Set Wavin setpoint to 7 C (effectively stops heating, since room temperature is always above 7 C)

## Prerequisites

- Wavin AHC-9000 ESPHome component installed and running
- Climate entities exposed in Home Assistant (e.g. `climate.master_bedroom`)
- [Versatile Thermostat](https://github.com/jmcollin78/versatile_thermostat) (HACS integration)

## Step 1: Create Input Boolean Helpers

In Home Assistant: **Settings > Devices & Services > Helpers > Create Helper > Toggle**

Create one toggle per zone:

| Helper                                | Zone              |
|---------------------------------------|-------------------|
| `input_boolean.master_bedroom_heating` | Master Bedroom   |
| `input_boolean.koekken_stue_heating`  | Koekken & Stue    |
| `input_boolean.badevaerelse_heating`   | Stort Badevaerelse|
| `input_boolean.entre_heating`         | Entre             |
| `input_boolean.gang_heating`          | Gang              |

## Step 2: Create Automations

In Home Assistant: **Settings > Automations & Scenes > Create Automation**

Create one automation per zone. Each automation listens for the input boolean toggling and sets the Wavin climate setpoint accordingly.

### Single-channel zone example (Master Bedroom)

```yaml
alias: "Wavin Master Bedroom Heating Demand"
triggers:
  - trigger: state
    entity_id: input_boolean.master_bedroom_heating
actions:
  - action: climate.set_temperature
    target:
      entity_id: climate.master_bedroom
    data:
      temperature: "{{ 35 if trigger.to_state.state == 'on' else 7 }}"
```

### Multi-channel zone example (Koekken & Stue)

For group climates that span multiple Wavin channels, target the group climate entity:

```yaml
alias: "Wavin Koekken & Stue Heating Demand"
triggers:
  - trigger: state
    entity_id: input_boolean.koekken_stue_heating
actions:
  - action: climate.set_temperature
    target:
      entity_id: climate.koekken_stue
    data:
      temperature: "{{ 35 if trigger.to_state.state == 'on' else 7 }}"
```

Repeat for each remaining zone (`badevaerelse`, `entre`, `gang`).

## Step 3: Configure VTherm

For each zone, create a new Versatile Thermostat:

1. **Type**: `over_switch`
2. **Switch entity**: The corresponding `input_boolean` (e.g. `input_boolean.master_bedroom_heating`)
3. **Temperature sensor**: Your room temperature sensor (this is the sensor VTherm uses to decide when to heat -- not the Wavin floor sensor)
4. **TPI coefficients**: Start with defaults (`coef_int: 0.6`, `coef_ext: 0.01`) and tune as needed

VTherm will now cycle the input boolean on/off using TPI, which triggers the automation to set Wavin's setpoint high (heat) or low (idle).

## Why This Approach?

- **No ESPHome firmware changes required** -- everything is configured in Home Assistant
- **No mode switching** -- avoids Modbus writes to change HEAT/OFF mode, reducing bus traffic
- **Simple and reliable** -- setpoint writes are the most basic Wavin operation
- **VTherm handles the intelligence** -- TPI algorithm, presets, window detection, presence detection, etc.
