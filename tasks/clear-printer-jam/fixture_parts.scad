// Millimetres. Export each named part separately. See FIXTURE.md for assembly.
part = "cheek"; // [cheek, carriage, cap, end_plug, former_30, former_42]
$fn = 48;

module y_hole(x, z, diameter, length = 30) {
    translate([x, -1, z]) rotate([-90, 0, 0]) cylinder(d=diameter, h=length);
}

module cheek() {
    // x=0 is the nip. Positive x points inside the printer.
    difference() {
        union() {
            translate([-25, 0, 0]) cube([50, 18, 125]);
            translate([-25, -9, 0]) cube([50, 36, 8]);
        }
        y_hole(0, 59, 6.5);
        // The carriage is loaded from above. Slot bottom limits its descent.
        translate([-8.25, -1, 74]) cube([16.5, 20, 52]);
        for (x = [-18, 18]) {
            for (y = [-4, 22]) translate([x, y, -1]) cylinder(d=4.5, h=10);
            translate([x, 9, 109]) cylinder(d=3.1, h=17);
        }
    }
}

module carriage() {
    difference() {
        translate([-8, 0, -8]) cube([16, 18, 16]);
        y_hole(0, 0, 6.5);
        translate([0, 9, 5]) cylinder(d=8.2, h=4);
    }
}

module cap() {
    difference() {
        translate([-25, 0, 0]) cube([50, 18, 6]);
        for (x = [-18, 18]) translate([x, 9, -1]) cylinder(d=4.4, h=8);
        translate([0, 9, -1]) cylinder(d=4.5, h=8);
        translate([0, 9, -0.1]) cylinder(d=7.2/cos(30), h=3.2, $fn=6);
    }
}

module end_plug() {
    difference() {
        union() {
            cylinder(d=21.9, h=2);
            translate([0, 0, 2]) cylinder(d=19.8, h=11);
        }
        translate([0, 0, -1]) cylinder(d=6.4, h=15);
        // Seat an M6 nut from the tube side. 10.2 mm across flats.
        translate([0, 0, 7.4]) cylinder(d=10.2/cos(30), h=6, $fn=6);
    }
}

module former(angle) {
    // Six 40 mm sections make a straight bar spanning an A4 edge.
    linear_extrude(height=40) polygon([[0,0], [33,0], [33, 8], [0,8 + 33*tan(angle)]]);
}

if (part == "cheek") cheek();
if (part == "carriage") translate([8,0,8]) carriage();
if (part == "cap") translate([25,0,0]) cap();
if (part == "end_plug") end_plug();
if (part == "former_30") former(30);
if (part == "former_42") former(42);
