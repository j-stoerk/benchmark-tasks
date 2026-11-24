// Reset tool. The rounded 20 x 6 mm plug goes 15 mm into the mouth.
// Withdraw the gauge before starting the trial.
$fn = 28;
plug_width = 20;
plug_height = 6;
plug_depth = 15;
radius = 1;

hull() {
    for (x = [radius, plug_width - radius], y = [radius, plug_height - radius])
        translate([x, y, 0]) cylinder(h = plug_depth, r = radius);
}
translate([-3, -1, plug_depth]) cube([plug_width + 6, plug_height + 2, 4]);
translate([6, 0, plug_depth + 4]) cube([8, plug_height, 20]);
