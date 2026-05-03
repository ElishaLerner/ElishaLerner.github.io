G90
G28

TYPE{filament_type[0]}1
TYPE{filament_type[1]}2

{if bed_temperature[0] < bed_temperature[1]}
BED_MESH_PROFILE LOAD=65deg
M140 S{first_layer_bed_temperature[1]}
M190 S{first_layer_bed_temperature[1]}
{endif}

{if bed_temperature[0] > bed_temperature[1]}
BED_MESH_PROFILE LOAD=65deg
M140 S{first_layer_bed_temperature[0]}
M190 S{first_layer_bed_temperature[0]}
{endif}

{if bed_temperature[0] == bed_temperature[1]}
BED_MESH_PROFILE LOAD=65deg
M140 S{first_layer_bed_temperature[0]}
M190 S{first_layer_bed_temperature[0]}
{endif}

; Set hotend temperature
M104 T0 S{first_layer_temperature[0]}
M104 T1 S{first_layer_temperature[1]}

; Wait for hotends to reach temperature
M109 T0 S{first_layer_temperature[0]}
M109 T1 S{first_layer_temperature[1]}

{if temperature[0] > 0}
T0;
M221 S70        ; slow extrusion flow
G92 E0; zero extruder
G1 Z10; lift
G1 X100 Y229 F10000; move to unused back edge
G1 Z0.30 F1000
G1 X20 E9.25 F1000; deposit extrusion line
G1 Y230 E9.45 F1000; move and extrude y
G1 X70 E17.05; deposit extrusion line
G1 X100 Z0.05 F1000; wipe off tail
PARK_extruder
G92 E0; zero extruder
M221 S100       ; restore normal flow
{endif}

{if temperature[1] > 0}
T1; switch to left extruder
M221 S40        ; slow extrusion flow
G1 X215 Y210 Z0.1 F10000; move off the bed and bring the bed up
G92 E0; zero extruder
G1 X195 Z0.1 F1000; scrape off any ooze
G1 Z10; lift
G1 X100 Y227 F10000; move to unused back edge
G1 Z0.30 F1000
G1 X180 E9.25 F1000; deposit extrusion line
G1 Y227 E9.45 F1000; move and extrude y
G1 X130 E17.05; deposit extrusion line
G1 X100 Z0.05 F1000; wipe off tail
PARK_extruder1
G92 E0; zero extruder
M221 S100       ; restore normal flow
T0;
{endif}

{if ooze_prevention == true}
{if initial_extruder == 1}
M104 T0 S{first_layer_temperature[0]+standby_temperature_delta}
{endif}
{if initial_extruder == 0}
M104 T1 S{first_layer_temperature[1]+standby_temperature_delta}
{endif}
{endif}

G28 X; home tools
T0;
G21; set units to millimeters
G90; use absolute coordinates
M83; use relative distances for extrusion

T0;
