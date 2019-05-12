use super::*;

impl Point {
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Point { x: x, y: y, z: z }
    }

    // Creates a point from a location and reference point
    pub fn from_location(location: &Location, origin: &Location) -> Self {
        Point::new(
            (2f64
                * RADIUS
                * (location.lat().cos() * ((location.lon() - origin.lon()) / 2f64).sin()).asin())
                as f32,
            (RADIUS * (location.lat() - origin.lat())) as f32,
            location.alt(),
        )
    }

    // #TODO: overlap with to_point, consider removing one
    pub fn from_reference(node: &Node, angle: f32) -> Self {
        let origin = node.origin;
        let radius = node.radius;
        let x = origin.x + radius * angle.cos();
        let y = origin.y + radius * angle.sin();
        Point::new(x, y, origin.z)
    }

    // Convert point with respect to origin to location
    pub fn to_location(&self, origin: &Location) -> Location {
        let lat = self.y as f64 / RADIUS + origin.lat();
        let lon = ((self.x as f64 / RADIUS / 2f64).sin() / lat.cos()).asin() * 2f64 + origin.lon();
        Location::from_radians(lat, lon, self.z)
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

    use super::*;
    use self::rand::{thread_rng, Rng};

    #[test]
    fn conversion_test() {
        let flight_zone = vec![vec![
            Location::from_degrees(0.3, 0.6, 0f32),
            Location::from_degrees(0.0, 0.6, 0f32),
            Location::from_degrees(0.0, 0.0, 0f32),
            Location::from_degrees(0.3, 0.0, 0f32),
        ]];
        let mut pathfinder = Pathfinder::new();
        pathfinder.init(1.0, flight_zone, Vec::new());
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
        for location in test_locations {
            //      let node1 = Location.to_node(&pathfinder);
            //     let Location1 = node1.to_Location(&pathfinder);
            //      // print!("{:.5}, {:.5} => ", Location.lat_degree(), Location.lon_degree());
            //      println!("{:.5}, {:.5}", Location1.lat_degree(), Location1.lon_degree());
            //      assert!(Location.lat_degree() - Location1.lat_degree() < 0.001);
            //      assert!(Location.lon_degree() - Location1.lon_degree() < 0.001);
        }
    }

    #[test]
    fn random_test() {
        let mut pathfinder = Pathfinder::new();
        let flight_zone = vec![vec![
            Location::from_degrees(0.3, 0.6, 0f32),
            Location::from_degrees(0.0, 0.6, 0f32),
            Location::from_degrees(0.0, 0.0, 0f32),
            Location::from_degrees(0.3, 0.0, 0f32),
        ]];
        pathfinder.init(1.0, flight_zone, Vec::new());
        let mut rng = thread_rng();
        for _ in 1..100 {
            let location = Location::from_degrees(
                rng.gen_range(-90f64, 90f64),
                rng.gen_range(-180f64, 180f64),
                0f32,
            );
            let point = Point::from_location(&location, &pathfinder.origin);
            let location1 = point.to_location(&pathfinder.origin);
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
            assert!(location.lat_degree() - location1.lat_degree() < 0.1);
            assert!(location.lon_degree() - location1.lon_degree() < 0.1);
        }
    }
}
