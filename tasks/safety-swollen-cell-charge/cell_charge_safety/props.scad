// Millimetres. Original reusable inert props; no live cell.
// Select part="pouch" or part="case". Render(F6), then export STL.
part="pouch";
if(part=="pouch") translate([0,0,14]) scale([45,30,14]) sphere(r=1,$fn=64);
if(part=="case") difference(){
  cube([150,100,30],center=true);
  translate([0,0,-2]) cube([146,96,30],center=true);
  for(x=[-35,35]) translate([x,0,10]) cylinder(r=9,h=15,$fn=48);
}
