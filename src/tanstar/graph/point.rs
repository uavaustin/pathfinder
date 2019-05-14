use super::*;

#[derive(Copy, Clone, Debug)]
pub struct Point {
    pub x: f32, // horizontal distance from origin in meters
    pub y: f32, // vertical distance from origin in meters
    pub z: f32,
}

impl From<(&Point, &Location)> for Location {
    // Convert point with respect to origin to location
    fn from((point, origin): (&Point, &Location)) -> Self {
        let lat = point.y as f64 / RADIUS + origin.lat();
        let lon = ((point.x as f64 / RADIUS / 2f64).sin() / lat.cos()).asin() * 2f64 + origin.lon();
        Self::from_radians(lat, lon, point.z)
    }
}

impl From<(&Location, &Location)> for Point {
    // Creates a point from a location and reference origin
    fn from((location, origin): (&Location, &Location)) -> Self {
        Self::new(
            (2f64
                * RADIUS
                * (location.lat().cos() * ((location.lon() - origin.lon()) / 2f64).sin()).asin())
                as f32,
            (RADIUS * (location.lat() - origin.lat())) as f32,
            location.alt(),
        )
    }
}

impl From<(&Node, f32)> for Point {
    // Create point from node and vertex angle
    fn from((node, angle): (&Node, f32)) -> Self {
        let origin = node.origin;
        let radius = node.radius;
        let x = origin.x + radius * angle.cos();
        let y = origin.y + radius * angle.sin();
        Self::new(x, y, origin.z)
    }
}

impl Location {
    // Create location using x-y distance from origin
    pub fn from_meters(x: f32, y: f32, alt: f32, origin: &Location) -> Self {
        (&Point::new(x, y, alt), origin).into()
    }
}

impl Point {
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x: x, y: y, z: z }
    }

    pub fn distance(&self, other: &Point) -> f32 {
        ((self.x - other.x).powi(2) + (self.y - other.y).powi(2)).sqrt() as f32
    }

    pub fn distance3d(&self, other: &Point) -> f32 {
        ((self.x - other.x).powi(2) + (self.y - other.y).powi(2) + (self.z - other.z).powi(2))
            .sqrt() as f32
    }
}

#[cfg(test)]
mod test {
    extern crate rand;

    use self::rand::{thread_rng, Rng};
    use super::*;

    #[test]
    fn conversion_test() {
        let flight_zone = vec![vec![
            Location::from_degrees(0.3, 0.6, 0f32),
            Location::from_degrees(0.0, 0.6, 0f32),
            Location::from_degrees(0.0, 0.0, 0f32),
            Location::from_degrees(0.3, 0.0, 0f32),
        ]];
        let pathfinder = Tanstar::create(5f32, flight_zone, Vec::new());
        let test_locations = vec![
            Location::from_degrees(30.32247, -97.6009, 0f32),
            Location::from_degrees(30.32307, -97.6005, 0f32),
            Location::from_degrees(30.32373, -97.6012, 0f32),
            Location::from_degrees(30.32366, -97.6019, 0f32),
            Location::from_degrees(30.32321, -97.6025, 0f32),
            Location::from_degrees(30.32521, -97.60230, 0f32),
            Location::from_degrees(30.32466, -97.59856, 0f32),
            Location::from_degrees(30.32107, -97.60032, 0f32),
            Location::from_degrees(30.32247, -97.60325, 0f32),
            Location::from_degrees(30.32473, -97.60410, 0f32),
        ];
        for loc in test_locations {
            let node1 = Node::from((&loc, &pathfinder.origin, 5f32));
            let new_loc = Location::from((&node1.origin, &pathfinder.origin));
            // print!("{:.5}, {:.5} => ", loc.lat_degree(), loc.lon_degree());
            println!("{:.5}, {:.5}", new_loc.lat_degree(), new_loc.lon_degree());
            assert!((loc.lat_degree() - new_loc.lat_degree()).abs() < 0.001);
            assert!((loc.lon_degree() - new_loc.lon_degree()).abs() < 0.001);
        }
    }

    #[test]
    fn random_test() {
        let flight_zone = vec![vec![
            Location::from_degrees(0.3, 0.6, 0f32),
            Location::from_degrees(0.0, 0.6, 0f32),
            Location::from_degrees(0.0, 0.0, 0f32),
            Location::from_degrees(0.3, 0.0, 0f32),
        ]];
        let pathfinder = Tanstar::create(5f32, flight_zone, Vec::new());
        let mut rng = thread_rng();
        for _ in 1..100 {
            let location = Location::from_degrees(
                rng.gen_range(-10f64, 10f64),
                rng.gen_range(-10f64, 10f64),
                0f32,
            );
            let point = Point::from((&location, &pathfinder.origin));
            let location1 = Location::from((&point, &pathfinder.origin));
            print!(
                "{:.5}, {:.5} => ",
                location.lat_degree(),
                location.lon_degree()
            );
            println!(
                "{:.5}, {:.5}",
                location1.lat_degree(),
                location1.lon_degree()
            );
            assert!((location.lat_degree() - location1.lat_degree()).abs() < 0.0001);
            assert!((location.lon_degree() - location1.lon_degree()).abs() < 0.0001);
        }
    }
}
