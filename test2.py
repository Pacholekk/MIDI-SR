import rtmidi

midi_in = rtmidi.MidiIn()
midi_out = rtmidi.MidiOut()

available_ports_in = midi_in.get_ports()
available_ports_out = midi_out.get_ports()

print("Available MIDI Input Ports:")
for i, port in enumerate(available_ports_in):
    print(f"{i+1}. {port}")

print("Available MIDI Output Ports:")
for i, port in enumerate(available_ports_out):
    print(f"{i+1}. {port}")
