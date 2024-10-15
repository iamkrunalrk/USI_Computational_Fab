$fa = 2;
$fs = 0.2;

// Create a solid cone with grid lines
module solid_cone_with_grid(cone_radius, cone_height, grid_spacing, grid_thickness) {
    difference() {
        cylinder(h = cone_height, r1 = cone_radius, r2 = 0, $fn = 100); // Solid cone
        for (x = [-cone_radius:grid_spacing:cone_radius]) {
            for (y = [-cone_radius:grid_spacing:cone_radius]) {
                translate([x, y, 0])
                    cylinder(h = cone_height, r = grid_thickness, $fn = 6); // Grid lines
            }
        }
    }
}

// Create a thin wall
module thin_wall(length, height, start_thickness, end_thickness) {
    for (i = [0:length]) {
        current_thickness = start_thickness - (start_thickness - end_thickness) * (i / length);
        translate([i, 0, 0]) cube([1, height, current_thickness]);
    }
}

// Create a thin hollow cylinder
module thin_hollow_cylinder(outer_radius, inner_radius, height) {
    difference() {
        cylinder(h = height, r = outer_radius, $fn = 200); // Outer cylinder
        translate([0, 0, 1]) cylinder(h = height, r = inner_radius, $fn = 200);
        translate([-10,0,0]) cube([20, 20, height+1]); // Inner cylinder (offset to avoid coincident faces)
    }
}

// Common base settings
module common_base() {
    translate([-11,-12,0]) cube([22,42,3]);
}

// Create solid cone with grid lines
module create_solid_cone() {
    translate([0, 0, 3])
    solid_cone_with_grid(10, 25, 1.0, 0.5);
}

// Create thin wall
module create_thin_wall() {
    translate([10, 13, 3])
    rotate([0, -90, 90])
    thin_wall(20, 20, 2, 0);
}

// Create thin hollow cylinder
module create_thin_hollow_cylinder() {
    translate([0, 25, 3])
    thin_hollow_cylinder(9, 8.5, 10);
}

// Create the additional geometry
module additional_geometry() {
    difference() {
        translate([0, 25, 12.5]) sphere(9);
        translate([0, 25, 12.5]) sphere(8.5);
        translate([-10, 25, 3]) cube([20, 20, 19]);
        translate([0, 25, 3]) cylinder(10, 10, 10);
    }
}

// Combine all components
common_base();
create_solid_cone();
create_thin_wall();
create_thin_hollow_cylinder();
additional_geometry();

// Additional shapes
difference() {
    translate([0, 28, 5]) sphere(5);
    translate([0, 28, 5]) sphere(4.5);
    translate([-5, 28.5, 0]) #cube([10,4.5,10]);
    translate([-5, 21.5, 3]) cube([10,4.5,7]);
    translate([1, 25, 3]) cube([4,5,7]);
}
