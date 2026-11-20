// Dimensions are in millimetres. The origin is on the bench at X 334, Y 0.
// Cut two 402 x 64 panels and two 362 x 64 panels from clear 3 mm stock.
// Seal the wet-side seams above a continuous waterproof film floor.

tray_x = 368;
tray_y = 402;
wall = 3;
height = 64;
floor = 0.36;

color([0.40, 0.65, 0.68, 0.30]) {
    translate([-tray_x / 2, -tray_y / 2, 0]) cube([tray_x, tray_y, floor]);

    for (side = [-1, 1]) {
        translate([side * (tray_x - wall) / 2, 0, height / 2])
            cube([wall, tray_y, height], center = true);

        translate([0, side * (tray_y - wall) / 2, height / 2])
            cube([tray_x - 2 * wall, wall, height], center = true);
    }
}
