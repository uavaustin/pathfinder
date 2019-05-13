use super::*;
use obj::Obstacle;
const THRESHOLD: f64 = 0.001;

//assert equal, equal practically because floating points suck for intersection
macro_rules! assert_eqp {
    ($x:expr, $y:expr, $d:expr) => {
        if !((($x - $y) as f64).abs() < $d) {
            //println!("{} vs {}", $x, $y);
            panic!();
        }
    };
}

//compare two vectors with tuple of 2 elements
fn assert_vec2_eqp(v1: &Vec<(f32, f32)>, v2: &Vec<(f32, f32)>) {
    for i in 0..v1.len() {
        let a = v1[i];
        let b = v2[i];
        println!("comparing {:?} with {:?}", a, b);
        assert_eqp!(a.0, b.0, THRESHOLD);
        assert_eqp!(a.1, b.1, THRESHOLD);
    }
}

//compare two vectors of tuple with 3 elements
fn assert_vec3_eqp(v1: &Vec<(f32, f32, f32)>, v2: &Vec<(f32, f32, f32)>) {
    for i in 0..v1.len() {
        let a = v1[i];
        let b = v2[i];
        println!("comparing {:?} with {:?}", a, b);
        assert_eqp!(a.0, b.0, THRESHOLD);
        assert_eqp!(a.1, b.1, THRESHOLD);
        assert_eqp!(a.2, b.2, THRESHOLD);
    }
}

fn assert_vec4_eqp(v1: &Vec<(f32, f32, f32, f32)>, v2: &Vec<(f32, f32, f32, f32)>) {
    for i in 0..v1.len() {
        let a = v1[i];
        let b = v2[i];
        println!("comparing {:?} with {:?}", a, b);
        assert_eqp!(a.0, b.0, THRESHOLD);
        assert_eqp!(a.1, b.1, THRESHOLD);
        assert_eqp!(a.2, b.2, THRESHOLD);
        assert_eqp!(a.3, b.3, THRESHOLD);
    }
}

fn assert_point_eq(a: &Point, b: &Point) {
    assert_eqp!(a.x, b.x, THRESHOLD);
    assert_eqp!(a.y, b.y, THRESHOLD);
}

fn dummy_origin() -> Location {
    Location::from_radians(0f64, 0f64, 0f32)
}

// Helper function to create an obstacle based on its position in transformed graph
fn obstacle_from_meters(x: f32, y: f32, radius: f32, height: f32) -> Obstacle {
    Obstacle::new(
        Location::from_meters(x, y, height, &dummy_origin()),
        radius,
        height,
    )
}

fn dummy_flyzones() -> Vec<Vec<Location>> {
    let a = Point::new(0f32, 0f32, 10f32);
    let b = Point::new(0f32, 400f32, 10f32);
    let c = Point::new(400f32, 400f32, 10f32);
    let d = Point::new(400f32, 0f32, 10f32);
    vec![points_to_flyzone(vec![a, b, c, d])]
}

fn dummy_pathfinder() -> Pathfinder {
    Pathfinder::create(1f32, dummy_flyzones(), Vec::new())
}

fn points_to_flyzone(points: Vec<Point>) -> Vec<Location> {
    let mut flyzone = Vec::new();
    for point in points {
        flyzone.push(Location::from((&point, &dummy_origin())));
    }
    flyzone
}

#[test]
fn flyzone_pathing() {
    let a = Point::new(40f32, 0f32, 10f32);
    let b = Point::new(40f32, 40f32, 10f32);
    let c = Point::new(0f32, 40f32, 10f32);
    let d = Point::new(0f32, 0f32, 10f32);
    let flyzones = vec![points_to_flyzone(vec![a, b, c, d])];
    let pathfinder = Pathfinder::create(1f32, flyzones, Vec::new());

    let e = Point::new(20f32, 20f32, 10f32);
    let f = Point::new(30f32, 30f32, 10f32);
    let g = Point::new(20f32, 50f32, 10f32);

    let h = Point::new(50f32, 50f32, 10f32);
    let i = Point::new(50f32, 0f32, 10f32);

    assert_eq!(bool::from(pathfinder.valid_path(&e, &f)), true);
    assert_eq!(bool::from(pathfinder.valid_path(&e, &g)), false);
    assert_eq!(bool::from(pathfinder.valid_path(&f, &g)), false);
    assert_eq!(bool::from(pathfinder.valid_path(&a, &b)), false);
    assert_eq!(bool::from(pathfinder.valid_path(&a, &h)), false);

    //here some points are outside of the flyzone; should this be a special case?
    //should we assume that the points we evaluate will always be inside the flyzone?
    assert_eq!(bool::from(pathfinder.valid_path(&h, &i)), true);
    assert_eq!(bool::from(pathfinder.valid_path(&h, &e)), false);
}

#[test]
fn flyzones_pathing() {
    let a = Point::new(40f32, 0f32, 10f32);
    let b = Point::new(40f32, 40f32, 10f32);
    let c = Point::new(0f32, 40f32, 10f32);
    let d = Point::new(0f32, 0f32, 10f32);

    let e = Point::new(30f32, 10f32, 10f32);
    let f = Point::new(30f32, 30f32, 10f32);
    let g = Point::new(10f32, 30f32, 10f32);
    let h = Point::new(10f32, 10f32, 10f32);

    let flyzone1 = points_to_flyzone(vec![a, b, c, d]);
    let flyzone2 = points_to_flyzone(vec![e, f, g, h]);

    let flyzones = vec![flyzone1, flyzone2];

    let mut pathfinder = Pathfinder::new();
    pathfinder.init(1f32, flyzones, Vec::new());

    //test breaks with multiple flyzones; must declare every flyzone from meters at (0,0)
    /*let i = Point::new(15f32, 15f32, 10f32);
    let j = Point::new(25f32, 25f32, 10f32);
    let k = Point::new(35f32, 5f32, 10f32);
    let l = Point::new(50f32, 50f32, 10f32);
    let m = Point::new(35f32, 25f32, 10f32);

    assert_eq!(pathfinder.valid_path(&i, &j), true);
    assert_eq!(pathfinder.valid_path(&i, &k), false);
    assert_eq!(pathfinder.valid_path(&i, &l), false);
    assert_eq!(pathfinder.valid_path(&k, &l), false);
    assert_eq!(pathfinder.valid_path(&k, &m), true);
    */
}

#[test]
// https://www.geogebra.org/graphing/mfqccnkb
fn obstacles_pathing() {
    let a = Point::new(20f32, 40f32, 10f32);
    let b = Point::new(20f32, 1f32, 10f32);
    let c = Point::new(20f32, 60f32, 10f32);
    let d = Point::new(60f32, 20f32, 10f32);
    let e = Point::new(20f32, 30f32, 10f32);

    let ob = obstacle_from_meters(20f32, 20f32, 20f32, 20f32);
    let obstacles = vec![ob];

    let mut pathfinder = Pathfinder::new();
    pathfinder.init(1f32, dummy_flyzones(), obstacles);
    pathfinder.set_buffer(0f32);

    match pathfinder.valid_path(&a, &b) {
        PathValidity::Flyover(threshold) => assert_eq!(threshold, 20f32),
        _ => panic!(),
    }
    match pathfinder.valid_path(&c, &d) {
        PathValidity::Flyover(threshold) => assert_eq!(threshold, 0f32),
        _ => panic!(),
    }
    match pathfinder.valid_path(&c, &e) {
        PathValidity::Flyover(threshold) => assert_eq!(threshold, 20f32),
        _ => panic!(),
    }
}

#[test]
fn intersects_circle() {
    //Desmos Visual: https://www.desmos.com/calculator/fxknkpinao

    //Test Object - Desmos Eq 1
    let pathfinder = dummy_pathfinder();
    let ob = obstacle_from_meters(15f32, 0f32, 5f32, 20f32);

    //Check intersections of line from (0,0) to (30,0) with circle of radius 5 centered at (15,0)
    //2 sol - Desmos Eq 2
    let a = Point::new(0f32, 0f32, 0f32);
    let b = Point::new(30f32, 0f32, 0f32);

    let (c1, c2) = circular_intersect(&pathfinder.origin, &a, &b, &ob);
    assert!(c1.is_some());
    assert_eq!(c1.unwrap().x, 10f32);
    assert_eq!(c1.unwrap().y, 0f32);

    assert!(c2.is_some());
    assert_eq!(c2.unwrap().x, 20f32);
    assert_eq!(c2.unwrap().y, 0f32);

    //Check intersections of line from (0,5) to (30,5) with circle of radius 5 centered at (15,0)
    //1 sol - Desmos Eq 3
    let d = Point::new(0f32, 5f32, 0f32);
    let e = Point::new(30f32, 5f32, 0f32);

    let (f1, f2) = circular_intersect(&pathfinder.origin, &d, &e, &ob);
    assert!(f1.is_some());
    assert_eq!(f1.unwrap().x, 15f32);
    assert_eq!(f1.unwrap().y, 5f32);

    assert!(f2.is_none());

    //Check intersections of line from (10,-5) to (10,5) with circle of radius 5 centered at (15,0)
    //1 sol - Desmos Eq 4
    let g = Point::new(10f32, -5f32, 0f32);
    let h = Point::new(10f32, 5f32, 0f32);

    let (i1, i2) = circular_intersect(&pathfinder.origin, &g, &h, &ob);
    assert!(i1.is_some());
    assert_eq!(i1.unwrap().x, 10f32);
    assert_eq!(i1.unwrap().y, 0f32);

    assert!(i2.is_none());

    //Check intersections of line from (10,-5) to (20,5) , y = x-15, with circle of radius 5 centered at (15,0)
    //2 sol - Desmos Eq 5
    let j = Point::new(10f32, -5f32, 0f32);
    let k = Point::new(20f32, 5f32, 0f32);

    let (l1, l2) = circular_intersect(&pathfinder.origin, &j, &k, &ob);
    assert!(l1.is_some());
    assert_eq!((l1.unwrap().x * 1000.0).round() / 1000.0, 11.464f32); //Rounded to 3 decimal
    assert_eq!((l1.unwrap().y * 1000.0).round() / 1000.0, -3.536f32); //Rounded to 3 decimal

    assert!(l2.is_some());
    assert_eq!((l2.unwrap().x * 1000.0).round() / 1000.0, 18.536f32);
    assert_eq!((l2.unwrap().y * 1000.0).round() / 1000.0, 3.536f32);

    //Check intersections of line from (10,10) to (15,-10) with circle of radius 5 centered at (15,0)
    //2 sol - Desmos Eq 6
    let m = Point::new(10f32, 10f32, 0f32);
    let n = Point::new(15f32, -10f32, 0f32);

    let (o1, o2) = circular_intersect(&pathfinder.origin, &m, &n, &ob);
    assert!(o1.is_some());
    assert_eq!((o1.unwrap().x * 1000.0).round() / 1000.0, 11.587f32); //Rounded to 3 decimal
    assert_eq!((o1.unwrap().y * 1000.0).round() / 1000.0, 3.654f32); //Rounded to 3 decimal

    assert!(o2.is_some());
    assert_eq!((o2.unwrap().x * 1000.0).round() / 1000.0, 13.708f32);
    assert_eq!((o2.unwrap().y * 1000.0).round() / 1000.0, -4.83f32);
}

#[test]
fn intersection_distance() {
    let ax = Point::new(0f32, 0f32, 0f32);
    let ay = Point::new(30f32, 0f32, 0f32);

    let bx = Point::new(10f32, 0f32, 0f32);
    let by = Point::new(20f32, 0f32, 0f32);

    let ob = obstacle_from_meters(15f32, 0f32, 5f32, 20f32);
    let pathfinder = Pathfinder::create(1f32, dummy_flyzones(), Vec::new());

    //intercepts at (10,0), (20,0)
    assert_eq!(
        intersect_distance(&ax, &ay, &Point::from((&ob.location, &dummy_origin()))).2,
        0f32
    );
    let result = perpendicular_intersect(&pathfinder.origin, &ax, &ay, &ob);
    println!("{:?} {:?}", result.0.unwrap(), result.1.unwrap());
    assert_point_eq(&result.0.unwrap(), &bx);
    assert_point_eq(&result.1.unwrap(), &by);
}

#[test]
fn circular_intersection() {
    //Desmos Visual: https://www.desmos.com/calculator/zkfgbbexkm

    //Check intersections of line from (0,0) to (30,0) with circle of radius 5 centered at (15,0)
    //2 sol
    let a = Point::new(0f32, 0f32, 0f32);
    let b = Point::new(30f32, 0f32, 0f32);

    let ob = obstacle_from_meters(15f32, 0f32, 5f32, 20f32);

    let pathfinder = dummy_pathfinder();
    let (c1, c2) = perpendicular_intersect(&pathfinder.origin, &a, &b, &ob);
    assert!(c1.is_some());
    assert_eq!(c1.unwrap().y, 0f32);
    assert_eq!(c1.unwrap().x, 10f32);

    assert!(c2.is_some());
    assert_eq!(c2.unwrap().y, 0f32);
    assert_eq!(c2.unwrap().x, 20f32);

    //Check intersections of line from (0,5) to (30,5) with circle of radius 5 centered at (15,0)
    //intersects at 1 point, should be considered valid
    let d = Point::new(0f32, 5f32, 0f32);
    let e = Point::new(30f32, 5f32, 0f32);

    let (f1, f2) = perpendicular_intersect(&pathfinder.origin, &d, &e, &ob);
    assert!(f1.is_none());
    assert!(f2.is_none());

    //Check intersections of line from (0,5) to (15,5) with circle of radius 5 centered at (15,0)
    //intersects at 1 point, should be considered valid
    let g = Point::new(10f32, -5f32, 0f32);
    let h = Point::new(10f32, 5f32, 0f32);

    let (i1, i2) = perpendicular_intersect(&pathfinder.origin, &g, &h, &ob);
    assert!(i1.is_none());
    //assert_eq!(i1.unwrap().y, 15f32);
    //assert_eq!(i1.unwrap().x, 5f32);

    assert!(i2.is_none());

    //should intersect at two points
    let j = Point::new(8f32, -2f32, 0f32);
    let k = Point::new(16f32, 6f32, 0f32);

    let (l1, l2) = perpendicular_intersect(&pathfinder.origin, &j, &k, &ob);
    assert!(l1.is_some());

    assert_eqp!(l1.unwrap().y, 0f32, 0.0001);
    assert_eqp!(l1.unwrap().x, 10f32, 0.0001);

    assert!(l2.is_some());
    assert_eqp!(l2.unwrap().y, 5f32, 0.0001);
    assert_eqp!(l2.unwrap().x, 15f32, 0.0001);

    //should intersect at two points
    let m = Point::new(8f32, 4f32, 0f32);
    let n = Point::new(30f32, -6f32, 0f32);

    let (o1, o2) = perpendicular_intersect(&pathfinder.origin, &m, &n, &ob);
    assert_eqp!(o1.unwrap().x, 10.807f32, 0.001);
    assert_eqp!(o1.unwrap().y, 2.724f32, 0.001);
    assert_eqp!(o2.unwrap().x, 19.809f32, 0.001);
    assert_eqp!(o2.unwrap().y, -1.368f32, 0.001);
}

/*
#[test]
fn obstacle_flyover() {
    //Graphical Visualization: https://www.geogebra.org/3d/a55hmxfy
    let a = Point::new(10f32, 10f32, 10f32);
    let b = Point::new(10f32, 40f32, 10f32);
    let c = Point::new(10f32, 30f32, 30f32);

    let d = Point::new(10f32, 40f32, 25f32);
    let e = Point::new(10f32, 10f32, 25f32);
    let f = Point::new(20f32, 40f32, 30f32);
    let g = Point::new(10f32, 30f32, 40f32);
    let ob = obstacle_from_meters(10f32, 25f32, 5f32, 20f32);
    let obstacles = vec![ob];
    let pathfinder = Pathfinder::create(1f32, dummy_flyzones(), obstacles);
    assert_eq!(bool::from(pathfinder.valid_path(&a, &b)), false);
    assert_eq!(bool::from(pathfinder.valid_path(&a, &d)), false);
    assert_eq!(bool::from(pathfinder.valid_path(&e, &b)), false);
    assert_eq!(bool::from(pathfinder.valid_path(&b, &e)), false);

    assert_eq!(bool::from(pathfinder.valid_path(&d, &e)), true);
    assert_eq!(bool::from(pathfinder.valid_path(&a, &c)), true);
    assert_eq!(bool::from(pathfinder.valid_path(&a, &g)), true);

    assert_eq!(bool::from(pathfinder.valid_path(&a, &f)), false);
}
*/

#[test]
fn generate_graph_test() {
    let a = Point::new(40f32, 0f32, 0f32);
    let b = Point::new(40f32, 40f32, 0f32);
    let c = Point::new(0f32, 40f32, 0f32);
    let d = Point::new(0f32, 0f32, 0f32);
    let flyzones = vec![points_to_flyzone(vec![a, b, c, d])];
    let obstacles = vec![
        obstacle_from_meters(10f32, 20f32, 10f32, 10f32),
        obstacle_from_meters(30f32, 20f32, 10f32, 10f32),
    ];
    let mut pathfinder = Pathfinder::new();
    pathfinder.init(5f32, flyzones, obstacles);
}

#[test]
// https://www.geogebra.org/graphing/hbtydqcz
fn same_radius_test() {
    let pathfinder = Pathfinder::create(1f32, dummy_flyzones(), Vec::new());

    let n1 = Node::new(Point::new(30_f32, 30_f32, 0_f32), 1_f32, 0_f32);
    let n2 = Node::new(Point::new(20_f32, 30_f32, 0_f32), 1_f32, 0_f32);
    let a1 = Rc::new(n1);
    let b1 = Rc::new(n2);
    let expected = vec![
        (PI / 2_f32, PI / 2_f32, 10f32, 0f32),
        (-PI / 2_f32, -PI / 2_f32, 10f32, 0f32),
        (101.537 * PI / 180f32, -78.463 * PI / 180f32, 9.798, 0f32),
        (-101.537 * PI / 180f32, 78.463 * PI / 180f32, 9.798, 0f32),
    ];
    assert_vec4_eqp(&pathfinder.find_path(&a1, &b1).0, &expected);
}

#[test]
// https://www.geogebra.org/graphing/nkjxtwrx
fn same_radius_offset_test() {
    let pathfinder = Pathfinder::create(1f32, dummy_flyzones(), Vec::new());
    let n1 = Node::new(Point::new(20_f32, 20_f32, 0_f32), 5_f32, 0_f32);
    let n2 = Node::new(Point::new(30_f32, 30_f32, 0_f32), 5_f32, 0_f32);
    let a1 = Rc::new(n1);
    let b1 = Rc::new(n2);

    let expected = vec![
        (7_f32 * PI / 4_f32, 7_f32 * PI / 4_f32, 200f32.sqrt(), 0f32),
        (
            -5_f32 * PI / 4_f32,
            -5_f32 * PI / 4_f32,
            200f32.sqrt(),
            0f32,
        ),
        (0_f32, -PI, 10f32, 0f32),
        (-3_f32 * PI / 2_f32, 3_f32 * PI / 2_f32, 10f32, 0f32),
    ];

    assert_vec4_eqp(&pathfinder.find_path(&a1, &b1).0, &expected);
}

#[test]
fn overlap_test() {
    let pathfinder = Pathfinder::create(1f32, dummy_flyzones(), Vec::new());
    let n3 = Node::new(Point::new(15_f32, 10_f32, 0_f32), 5_f32, 0_f32);
    let n4 = Node::new(Point::new(20_f32, 10_f32, 0_f32), 4_f32, 0_f32);
    let c = Rc::new(n3);
    let d = Rc::new(n4);
    let expected = vec![
        (4.913799976f32, 4.913799976f32, 4.898979486f32, 0f32),
        (-4.913799976f32, -4.913799976f32, 4.898979486f32, 0f32),
    ];
    assert_vec4_eqp(&pathfinder.find_path(&c, &d).0, &expected);
}

#[test]
fn sentinel_test() {
    let pathfinder = Pathfinder::create(1f32, dummy_flyzones(), Vec::new());
    let n3 = Node::new(Point::new(15_f32, 10_f32, 0_f32), 5_f32, 0_f32);
    let n4 = Node::new(Point::new(20_f32, 10_f32, 0_f32), 5_f32, 0_f32);
    let c = Rc::new(n3);
    let d = Rc::new(n4);
    let expected = vec![
        (PI / 3f32, 2f32 * PI / 3f32),
        (-PI / 3f32, 4f32 * PI / 3f32),
        (-5f32 * PI / 3f32, -2f32 * PI / 3f32),
        (5f32 * PI / 3f32, -4f32 * PI / 3f32),
    ];
    println!("{:?}", expected);
    assert_vec2_eqp(&pathfinder.find_path(&c, &d).1.unwrap(), &expected);
}

#[test]
fn different_radius_no_overlap_test() {
    let pathfinder = Pathfinder::create(1f32, dummy_flyzones(), Vec::new());
    let n5 = Node::new(Point::new(20_f32, 10_f32, 0_f32), 2_f32, 0_f32);
    let n6 = Node::new(Point::new(12_f32, 10_f32, 0_f32), 1_f32, 0_f32);
    let e = Rc::new(n5);
    let f = Rc::new(n6);
    let expected = vec![
        (1.6961242, 1.6961241, 63f32.sqrt(), 0f32),
        (-1.6961241, -1.6961241, 63f32.sqrt(), 0f32),
        (1.9551932, -1.1863995, 55f32.sqrt(), 0f32),
        (-1.955193, 1.1863995, 55f32.sqrt(), 0f32),
    ];
    assert_vec4_eqp(&pathfinder.find_path(&e, &f).0, &expected);
}

/*
#[test]
//all tangents are flying over an obstacle. returns threshold appropriately.
//https://www.geogebra.org/graphing/ufegkqcv
fn different_radius_no_overlap_all_flyover_test() {
    let obs = obstacle_from_meters(16f32, 10f32, 1.8f32, 20f32);
    let pathfinder = Pathfinder::create(1f32, dummy_flyzones(), vec![obs]);
    let n5 = Node::new(Point::new(20_f32, 10_f32, 30_f32), 2_f32, 0_f32);
    let n6 = Node::new(Point::new(12_f32, 10_f32, 30_f32), 1_f32, 0_f32);
    let e = Rc::new(n5);
    let f = Rc::new(n6);
    let expected = vec![
        (
            (1_f32 / 8_f32).acos(),
            (1_f32 / 8_f32).acos(),
            63f32.sqrt(),
            20f32,
        ),
        (
            -(1_f32 / 8_f32).acos(),
            -(1_f32 / 8_f32).acos(),
            63f32.sqrt(),
            20f32,
        ),
        (
            (3_f32 / 8_f32).acos(),
            -PI + (3_f32 / 8_f32).acos(),
            55f32.sqrt(),
            20f32,
        ),
        (
            -(3_f32 / 8_f32).acos(),
            PI - (3_f32 / 8_f32).acos(),
            55f32.sqrt(),
            20f32,
        ),
    ];
    assert_vec4_eqp(&pathfinder.find_path(&e, &f).0, &expected);
}
*/

/*
//one tangent is flying over an obstacle. returns threshold appropriately.
//https://www.geogebra.org/graphing/twuxqprk
fn different_radius_no_overlap_one_flyover_test() {
    let obs = obstacle_from_meters(16f32, 12f32, 1f32, 20f32);
    let pathfinder = Pathfinder::create(1f32, dummy_flyzones(), vec![obs]);
    let n5 = Node::new(Point::new(20_f32, 10_f32, 30_f32), 2_f32, 0_f32);
    let n6 = Node::new(Point::new(12_f32, 10_f32, 30_f32), 1_f32, 0_f32);
    let e = Rc::new(n5);
    let f = Rc::new(n6);
    let expected = vec![
        (
            (1_f32 / 8_f32).acos(),
            (1_f32 / 8_f32).acos(),
            63f32.sqrt(),
            20f32,
        ),
        (
            -(1_f32 / 8_f32).acos(),
            -(1_f32 / 8_f32).acos(),
            63f32.sqrt(),
            0f32,
        ),
        (
            (3_f32 / 8_f32).acos(),
            -PI + (3_f32 / 8_f32).acos(),
            55f32.sqrt(),
            0f32,
        ),
        (
            -(3_f32 / 8_f32).acos(),
            PI - (3_f32 / 8_f32).acos(),
            55f32.sqrt(),
            0f32,
        ),
    ];
    assert_vec4_eqp(&pathfinder.find_path(&e, &f).0, &expected);
}
*/

#[test]
fn virtualize_flyzone_square() {
    let origin = Location::from_degrees(0f64, 0f64, 0f32);
    let a = Location::from((&Point::new(0f32, 0f32, 10f32), &origin));
    let b = Location::from((&Point::new(20f32, 0f32, 10f32), &origin));
    let c = Location::from((&Point::new(20f32, 20f32, 10f32), &origin));
    let d = Location::from((&Point::new(0f32, 20f32, 10f32), &origin));
    let test_flyzone = vec![vec![d, c, b, a]];
    let mut pathfinder = Pathfinder::create(1f32, test_flyzone, Vec::new());
    let node_a = Point::new(5f32, 5f32, 0f32);
    let node_b = Point::new(15f32, 5f32, 0f32);
    let node_c = Point::new(15f32, 15f32, 0f32);
    let node_d = Point::new(5f32, 15f32, 0f32);
    let expected = vec![node_d, node_c, node_b, node_a];
    for i in 0..4 {
        assert_point_eq(&pathfinder.nodes[i].borrow().origin, &expected[i]);
    }
    let test_flyzone = vec![vec![a, b, c, d]];
    pathfinder.set_flyzone(test_flyzone);
    for i in 0..4 {
        assert_point_eq(&pathfinder.nodes[i].borrow().origin, &expected[i]);
    }
}

#[test]
fn virtualize_flyzone_plus() {
    let origin = Location::from_degrees(0f64, 0f64, 0f32);
    let a = Location::from((&Point::new(20f32, 0f32, 10f32), &origin));
    let b = Location::from((&Point::new(40f32, 0f32, 10f32), &origin));
    let c = Location::from((&Point::new(40f32, 20f32, 10f32), &origin));
    let d = Location::from((&Point::new(60f32, 20f32, 10f32), &origin));
    let e = Location::from((&Point::new(60f32, 40f32, 10f32), &origin));
    let f = Location::from((&Point::new(40f32, 40f32, 10f32), &origin));
    let g = Location::from((&Point::new(40f32, 60f32, 10f32), &origin));
    let h = Location::from((&Point::new(20f32, 60f32, 10f32), &origin));
    let i = Location::from((&Point::new(20f32, 40f32, 10f32), &origin));
    let j = Location::from((&Point::new(0f32, 40f32, 10f32), &origin));
    let k = Location::from((&Point::new(0f32, 20f32, 10f32), &origin));
    let l = Location::from((&Point::new(20f32, 20f32, 10f32), &origin));
    let test_flyzone = vec![vec![l, k, j, i, h, g, f, e, d, c, b, a]];
    let mut pathfinder = Pathfinder::create(1f32, test_flyzone, Vec::new());
    let node_a = Point::new(25f32, 5f32, 0f32);
    let node_b = Point::new(35f32, 5f32, 0f32);
    let node_c = Point::new(
        40f32 + (25f32 / 2f32).sqrt(),
        20f32 - (25f32 / 2f32).sqrt(),
        0f32,
    );
    let node_d = Point::new(55f32, 25f32, 0f32);
    let node_e = Point::new(55f32, 35f32, 0f32);
    let node_f = Point::new(
        40f32 + (25f32 / 2f32).sqrt(),
        40f32 + (25f32 / 2f32).sqrt(),
        0f32,
    );
    let node_g = Point::new(35f32, 55f32, 0f32);
    let node_h = Point::new(25f32, 55f32, 0f32);
    let node_i = Point::new(
        20f32 - (25f32 / 2f32).sqrt(),
        40f32 + (25f32 / 2f32).sqrt(),
        0f32,
    );
    let node_j = Point::new(5f32, 35f32, 0f32);
    let node_k = Point::new(5f32, 25f32, 0f32);
    let node_l = Point::new(
        20f32 - (25f32 / 2f32).sqrt(),
        20f32 - (25f32 / 2f32).sqrt(),
        0f32,
    );
    let expected = vec![
        node_l, node_k, node_j, node_i, node_h, node_g, node_f, node_e, node_d, node_c, node_b,
        node_a,
    ];
    for i in 0..11 {
        assert_point_eq(&pathfinder.nodes[i].borrow().origin, &expected[i]);
    }
    let test_flyzone = vec![vec![a, b, c, d, e, f, g, h, i, j, k, l]];
    pathfinder.set_flyzone(test_flyzone);
    for i in 0..4 {
        assert_point_eq(&pathfinder.nodes[i].borrow().origin, &expected[i]);
    }
}

#[test]
fn virtualize_flyzone_linear() {
    let origin = Location::from_degrees(0f64, 0f64, 0f32);
    let a = Location::from((&Point::new(0f32, 0f32, 10f32), &origin));
    let b = Location::from((&Point::new(20f32, 0f32, 10f32), &origin));
    let c = Location::from((&Point::new(20f32, 10f32, 10f32), &origin));
    let d = Location::from((&Point::new(20f32, 20f32, 10f32), &origin));
    let e = Location::from((&Point::new(0f32, 20f32, 10f32), &origin));
    let test_flyzone = vec![vec![e, d, c, b, a]];
    let mut pathfinder = Pathfinder::create(1f32, test_flyzone, Vec::new());
    let node_a = Point::new(5f32, 5f32, 0f32);
    let node_b = Point::new(15f32, 5f32, 0f32);
    let node_c = Point::new(15f32, 15f32, 0f32);
    let node_d = Point::new(5f32, 15f32, 0f32);
    let expected = vec![node_d, node_c, node_b, node_a];
    for i in 0..4 {
        assert_point_eq(&pathfinder.nodes[i].borrow().origin, &expected[i]);
    }
    let test_flyzone = vec![vec![a, b, c, d, e]];
    pathfinder.set_flyzone(test_flyzone);
    for i in 0..4 {
        assert_point_eq(&pathfinder.nodes[i].borrow().origin, &expected[i]);
    }
}

#[test]
fn virtualize_flyzone_small_angle() {
    let origin = Location::from_degrees(0f64, 0f64, 0f32);
    let a = Location::from((&Point::new(10f32, 0f32, 10f32), &origin));
    let b = Location::from((&Point::new(30f32, 0f32, 10f32), &origin));
    let c = Location::from((&Point::new(30f32, 20f32, 10f32), &origin));
    let d = Location::from((&Point::new(10f32, 20f32, 10f32), &origin));
    let e = Location::from((&Point::new(10f32, 11f32, 10f32), &origin));
    let f = Location::from((&Point::new(0f32, 10f32, 10f32), &origin));
    let g = Location::from((&Point::new(10f32, 9f32, 10f32), &origin));
    let test_flyzone = vec![vec![g, f, e, d, c, b, a]];
    let mut pathfinder = Pathfinder::create(1f32, test_flyzone, Vec::new());
    let node_a = Point::new(15f32, 5f32, 0f32);
    let node_b = Point::new(25f32, 5f32, 0f32);
    let node_c = Point::new(25f32, 15f32, 0f32);
    let node_d = Point::new(15f32, 15f32, 0f32);
    let node_f = Point::new(6.2927, 5.6450, 0f32);
    let node_e = Point::new(6.2927, 14.3550, 0f32);
    let expected = vec![node_f, node_e, node_d, node_c, node_b, node_a];
    for i in 0..6 {
        assert_point_eq(&pathfinder.nodes[i].borrow().origin, &expected[i]);
    }
    let test_flyzone = vec![vec![a, b, c, d, e, f, g]];
    pathfinder.set_flyzone(test_flyzone);
    for i in 0..6 {
        assert_point_eq(&pathfinder.nodes[i].borrow().origin, &expected[i]);
    }
}

/*#[test]
fn sentinel_vertex_test() {
    let a = Point::new(0f32, 0f32, 0f32).to_location(&origin);
    let b = Point::new(0f32, 5f32, 0f32).to_location(&origin);
    let a = Point::new(5f32, 0f32, 0f32).to_location(&origin);
    let test_flyzone = vec![vec![a, b, c]];
    let mut pathfinder = Pathfinder::create(1f32, test_flyzone, Vec::new());
    let origin = Point::new(4f32, 1f32, 0f32);
    let node = Node::new(origin, 1f32, 2f32);

    vertex_al = (3/2*PI, None);
    vertex_bl =
    vertex.angle =
    let expected_vertices = []
}*/
