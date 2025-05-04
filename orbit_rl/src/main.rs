use ggez::{Context, ContextBuilder, GameResult, GameError};
use ggez::conf::{WindowMode, WindowSetup};
use ggez::event::{self, EventHandler};
use ggez::graphics::{self, Color};
use ggez::glam;
use nalgebra::Vector2;
use ggez::input::keyboard::{KeyCode, KeyInput};

// --- Type Aliases ---
type Vec2 = Vector2<f64>; // For physics calculations
type Point2 = glam::Vec2; // For ggez drawing (uses f32)

// --- Simulation Constants ---
// Units: AU, Solar Masses (M☉), Years
const G: f64 = 39.478; // Gravitational constant in AU^3 / (M☉ * year^2)
const DT: f64 = 0.001; // Time step in years (approx 8-9 hours)
const EPSILON_SQ: f64 = 1e-8; // Min squared distance (can be smaller with AU)
const THRUSTER_FORCE: f64 = 1.0; // Thrust - needs tuning with new units/G
const FUEL_CONSUMPTION_RATE: f64 = 0.5; // Fuel rate - needs tuning
const ROTATION_SPEED: f64 = std::f64::consts::PI; // Radians per second (remains ok)

// --- Graphics Constants ---
const SCREEN_WIDTH: f32 = 1200.0;
const SCREEN_HEIGHT: f32 = 900.0;
// Define zoom levels for the global view (in AU vertically)
const GLOBAL_ZOOM_LEVELS: [f32; 3] = [10.0, 6.0, 3.0]; // Zoom out, Default, Zoom in
const GLOBAL_VIEW_HEIGHT: f32 = 6.0; // View height in AU for global view
const LOCAL_VIEW_HEIGHT: f32 = 0.05; // Zoom in local view significantly

// Helper to convert simulation coords to screen coords FOR A SPECIFIC VIEWPORT
fn world_to_screen_coords(
    world_pos: Vec2,      // Position in simulation units (AU)
    view_center: Vec2,    // Center of the view in simulation units (AU)
    view_height: f32,     // Height of the view in simulation units (AU)
    screen_viewport: graphics::Rect, // The target rectangle on the screen
) -> Point2 {
    // Calculate scale based on desired view height and viewport height
    let scale = screen_viewport.h / view_height;
    // Calculate offset relative to the view center
    let offset_x = world_pos.x - view_center.x;
    let offset_y = world_pos.y - view_center.y;
    // Calculate position relative to the top-left of the viewport
    Point2::new(
        (offset_x as f32 * scale) + screen_viewport.w / 2.0,
        (-offset_y as f32 * scale) + screen_viewport.h / 2.0,
    )
    // Add the viewport's top-left corner to get final screen coords
    + glam::Vec2::new(screen_viewport.x, screen_viewport.y) // Convert point to Vec2
}

// --- Celestial Body Definition ---
#[derive(Debug, Clone)]
struct CelestialBody {
    name: String,
    mass: f64,
    radius: f64,
    position: Vec2,
    velocity: Vec2,
    acceleration: Vec2, // Added for Verlet integration
    color: Color,
}

// --- Spacecraft Definition ---
#[derive(Debug, Clone)]
struct Spacecraft {
    mass: f64,       // Current mass (can decrease if fuel is consumed)
    fuel: f64,       // Remaining fuel
    position: Vec2,
    velocity: Vec2,
    orientation: f64, // Angle in radians (0 = pointing right along +X axis)
    acceleration: Vec2, // Added for Verlet integration
}

impl Spacecraft {
    fn new(mass: f64, fuel: f64, position: Vec2, velocity: Vec2, orientation: f64) -> Self {
        Spacecraft {
            mass,
            fuel,
            position,
            velocity,
            orientation,
            acceleration: Vec2::zeros(), // Initialize acceleration
        }
    }

    // Applies main thrust, consumes fuel, and returns the force vector
    fn apply_thrust(&mut self, dt: f64) -> Vec2 {
        if self.fuel <= 0.0 {
            return Vec2::zeros(); // Out of fuel
        }

        let fuel_consumed_this_step = FUEL_CONSUMPTION_RATE * dt;
        let actual_fuel_consumed = self.fuel.min(fuel_consumed_this_step);

        self.fuel -= actual_fuel_consumed;
        // Optional: Reduce mass based on fuel consumed if desired
        // self.mass -= actual_fuel_consumed * FUEL_MASS_RATIO; // Define FUEL_MASS_RATIO if needed

        // Calculate thrust vector based on orientation
        let direction = Vec2::new(self.orientation.cos(), self.orientation.sin());
        let thrust = direction * THRUSTER_FORCE;

        // Optionally scale thrust if fuel consumed was less than requested (ran out mid-step)
        // let thrust_scale = actual_fuel_consumed / fuel_consumed_this_step;
        // thrust * thrust_scale
        thrust // Keep it simple for now
    }

    // Rotates the spacecraft
    fn rotate(&mut self, direction: f64, dt: f64) {
        self.orientation += direction * ROTATION_SPEED * dt;
        // Keep angle within 0 to 2*PI range
        self.orientation = self.orientation.rem_euclid(2.0 * std::f64::consts::PI);
    }

    // Updates velocity based on total force (gravity + thrust)
    fn update_velocity(&mut self, total_force: Vec2, dt: f64) {
        if self.mass <= 0.0 { return; }
        let acceleration = total_force / self.mass;
        self.velocity += acceleration * dt;
    }

    // Updates position based on velocity
    fn update_position(&mut self, dt: f64) {
        self.position += self.velocity * dt;
    }
}

impl CelestialBody {
    fn new(name: String, mass: f64, radius: f64, position: Vec2, velocity: Vec2, color: Color) -> Self {
        CelestialBody {
            name,
            mass,
            radius,
            position,
            velocity,
            acceleration: Vec2::zeros(), // Initialize acceleration
            color,
        }
    }

    // Calculates the gravitational force exerted BY 'other' ON 'self'
    fn calculate_force(&self, other: &CelestialBody) -> Vec2 {
        let distance_vec = other.position - self.position;
        let distance_sq = distance_vec.norm_squared();

        if distance_sq < EPSILON_SQ {
             return Vec2::zeros();
        }

        let force_magnitude = G * self.mass * other.mass / distance_sq;
        let force_direction = distance_vec / distance_sq.sqrt(); // Normalized vector

        force_direction * force_magnitude
    }
}

// --- Game State ---
struct GameState {
    bodies: Vec<CelestialBody>,
    ship: Spacecraft,
    is_rotating_left: bool,
    is_rotating_right: bool,
    is_thrusting: bool,
    global_zoom_level: usize, // Index into GLOBAL_ZOOM_LEVELS
}

impl GameState {
    fn new() -> GameResult<GameState> {
        // Solar System Data (Approximate, Scaled)
        let sun_mass = 1.0;
        let earth_mass = 3.0e-6;
        let mars_mass = 3.3e-7;
        let moon_mass = earth_mass * 0.0123;
        let earth_dist = 1.0;
        let mars_dist = 1.52;
        let moon_dist = 0.00257;

        // --- Define Absolute Initial States (Position in AU, Velocity in AU/Year) ---
        let sun_pos = Vec2::new(0.0, 0.0);
        let sun_vel = Vec2::new(0.0, 0.0);
        let earth_pos = Vec2::new(earth_dist, 0.0);
        let earth_speed = (G * sun_mass / earth_dist).sqrt();
        let earth_vel = Vec2::new(0.0, earth_speed);
        let mars_pos = Vec2::new(-mars_dist, 0.0);
        let mars_speed = (G * sun_mass / mars_dist).sqrt();
        let mars_vel = Vec2::new(0.0, -mars_speed);
        let moon_pos_relative = Vec2::new(0.0, moon_dist);
        let moon_speed_relative = (G * earth_mass / moon_dist).sqrt();
        let moon_vel_relative = Vec2::new(-moon_speed_relative, 0.0);
        let moon_pos = earth_pos + moon_pos_relative;
        let moon_vel = earth_vel + moon_vel_relative;
        let ship_dist_relative_earth = 0.01;
        let ship_pos = earth_pos + Vec2::new(0.0, ship_dist_relative_earth);
        let ship_vel = earth_vel * 0.98;

        let bodies = vec![
            CelestialBody::new(
                "Sun".to_string(), sun_mass, 0.015, // Visually larger than planets, but small in AU
                sun_pos, sun_vel, Color::YELLOW,
            ),
            CelestialBody::new(
                "Earth".to_string(), earth_mass, 0.003, // Visible dot
                earth_pos, earth_vel, Color::BLUE,
            ),
            CelestialBody::new(
                "Mars".to_string(), mars_mass, 0.002, // Smaller visible dot
                mars_pos, mars_vel, Color::RED,
            ),
            CelestialBody::new(
                "Moon".to_string(), moon_mass, 0.0005, // Even tinier visible dot
                moon_pos, moon_vel, Color::from_rgb(128, 128, 128),
            ),
        ];

        let initial_ship = Spacecraft::new(
            1e-9, // Mass relative to M☉ (e.g., a few tons)
            100.0, // Initial fuel
            ship_pos,
            ship_vel,
            std::f64::consts::PI / 2.0, // Pointing 'up' (+Y)
        );

        // Create initial state struct BEFORE calculating accelerations
        let mut initial_state = GameState {
            bodies,
            ship: initial_ship,
            is_rotating_left: false,
            is_rotating_right: false,
            is_thrusting: false,
            global_zoom_level: 1, // Start at default zoom level (index 1 = 6.0 AU)
        };

        // --- Calculate Initial Accelerations (Needed for Verlet) ---
        let num_bodies = initial_state.bodies.len();
        let mut initial_body_forces = vec![Vec2::zeros(); num_bodies];
        let mut initial_ship_gravity_force = Vec2::zeros();

        // Calculate initial body-body forces
        for i in 0..num_bodies {
            for j in (i + 1)..num_bodies {
                let force = initial_state.bodies[i].calculate_force(&initial_state.bodies[j]);
                initial_body_forces[i] += force;
                initial_body_forces[j] -= force;
            }
        }
        // Calculate initial body-ship forces
        for body in &initial_state.bodies {
            let distance_vec = body.position - initial_state.ship.position;
            let distance_sq = distance_vec.norm_squared();
            if distance_sq >= EPSILON_SQ {
                let force_magnitude = G * body.mass * initial_state.ship.mass / distance_sq;
                let force_direction = distance_vec / distance_sq.sqrt();
                initial_ship_gravity_force += force_direction * force_magnitude;
            }
        }
        // Calculate initial accelerations and store them
        for i in 0..num_bodies {
            if initial_state.bodies[i].mass > 0.0 {
                initial_state.bodies[i].acceleration = initial_body_forces[i] / initial_state.bodies[i].mass;
            }
        }
        if initial_state.ship.mass > 0.0 {
            // Note: Initial thrust is zero here, so only gravity contributes
            initial_state.ship.acceleration = initial_ship_gravity_force / initial_state.ship.mass;
        }

        Ok(initial_state)
    }
}

// --- Event Handler Implementation ---
impl EventHandler<GameError> for GameState {
    fn update(&mut self, _ctx: &mut Context) -> GameResult {
        let dt = DT;
        let dt_squared = dt * dt;

        // --- Calculate Forces and Accelerations at time t ---
        let num_bodies = self.bodies.len();
        let mut body_forces_t = vec![Vec2::zeros(); num_bodies];
        let mut ship_gravity_force_t = Vec2::zeros();

        // Calculate body-body forces
        for i in 0..num_bodies {
            for j in (i + 1)..num_bodies {
                let force = self.bodies[i].calculate_force(&self.bodies[j]);
                body_forces_t[i] += force;
                body_forces_t[j] -= force;
            }
        }

        // Calculate body-ship forces
        for body in &self.bodies {
            let distance_vec = body.position - self.ship.position;
            let distance_sq = distance_vec.norm_squared();
            if distance_sq >= EPSILON_SQ {
                let force_magnitude = G * body.mass * self.ship.mass / distance_sq;
                let force_direction = distance_vec / distance_sq.sqrt();
                ship_gravity_force_t += force_direction * force_magnitude;
            }
        }

        // --- Input Handling (Rotation and Thrust Force at time t) ---
        let rotation_direction = if self.is_rotating_left { 1.0 }
                                 else if self.is_rotating_right { -1.0 }
                                 else { 0.0 };
        self.ship.rotate(rotation_direction, dt);

        let thrust_force_t = if self.is_thrusting { self.ship.apply_thrust(dt) } else { Vec2::zeros() };
        let total_ship_force_t = ship_gravity_force_t + thrust_force_t;

        // --- Update Positions (using v(t) and a(t)) ---
        // Note: a(t) is stored in the .acceleration field from the *previous* step
        for i in 0..num_bodies {
            let body = &mut self.bodies[i];
            body.position += body.velocity * dt + 0.5 * body.acceleration * dt_squared;
        }
        self.ship.position += self.ship.velocity * dt + 0.5 * self.ship.acceleration * dt_squared;

        // --- Calculate Forces and Accelerations at time t+dt (using new positions) ---
        let mut body_forces_t_dt = vec![Vec2::zeros(); num_bodies];
        let mut ship_gravity_force_t_dt = Vec2::zeros();

        // Recalculate body-body forces
        for i in 0..num_bodies {
            for j in (i + 1)..num_bodies {
                let force = self.bodies[i].calculate_force(&self.bodies[j]); // Uses updated positions
                body_forces_t_dt[i] += force;
                body_forces_t_dt[j] -= force;
            }
        }

        // Recalculate body-ship forces
        for body in &self.bodies {
            let distance_vec = body.position - self.ship.position; // Uses updated positions
            let distance_sq = distance_vec.norm_squared();
            if distance_sq >= EPSILON_SQ {
                let force_magnitude = G * body.mass * self.ship.mass / distance_sq;
                let force_direction = distance_vec / distance_sq.sqrt();
                ship_gravity_force_t_dt += force_direction * force_magnitude;
            }
        }

        // Assume thrust is constant during the step for simplicity, or recalculate if needed
        // Here we use the same thrust_force_t calculated earlier
        let total_ship_force_t_dt = ship_gravity_force_t_dt + thrust_force_t;

        // Debug Print Forces
        println!("Forces G={:.2} T={:.2} Tot={:.2}",
            ship_gravity_force_t_dt.norm(), thrust_force_t.norm(), total_ship_force_t_dt.norm());

        // Calculate accelerations at t+dt
        let mut body_accelerations_t_dt = vec![Vec2::zeros(); num_bodies];
        for i in 0..num_bodies {
            if self.bodies[i].mass > 0.0 {
                body_accelerations_t_dt[i] = body_forces_t_dt[i] / self.bodies[i].mass;
            }
        }
        let ship_acceleration_t_dt = if self.ship.mass > 0.0 {
            total_ship_force_t_dt / self.ship.mass
        } else {
            Vec2::zeros()
        };

        // --- Update Velocities (using average of a(t) and a(t+dt)) ---
        // Note: a(t) is still the value stored in the .acceleration field (from previous step)
        for i in 0..num_bodies {
            let body = &mut self.bodies[i];
            body.velocity += 0.5 * (body.acceleration + body_accelerations_t_dt[i]) * dt;
            // Store the *new* acceleration for the *next* step's position update
            body.acceleration = body_accelerations_t_dt[i];
        }

        self.ship.velocity += 0.5 * (self.ship.acceleration + ship_acceleration_t_dt) * dt;
        // Store the *new* acceleration for the *next* step's position update
        self.ship.acceleration = ship_acceleration_t_dt;

        // Debug print ship state
        println!("Ship Pos: ({:.2}, {:.2}), Vel: ({:.2}, {:.2})",
            self.ship.position.x, self.ship.position.y, self.ship.velocity.x, self.ship.velocity.y);

        Ok(())
    }

    fn draw(&mut self, ctx: &mut Context) -> GameResult {
        let mut canvas = graphics::Canvas::from_frame(ctx, graphics::Color::BLACK);

        // Define viewports for left (local) and right (global) halves
        let left_viewport = graphics::Rect::new(0.0, 0.0, SCREEN_WIDTH / 2.0, SCREEN_HEIGHT);
        let right_viewport = graphics::Rect::new(SCREEN_WIDTH / 2.0, 0.0, SCREEN_WIDTH / 2.0, SCREEN_HEIGHT);

        // --- Draw Local View (Left Side) ---
        let local_view_center = self.ship.position;
        let local_scale = left_viewport.h / LOCAL_VIEW_HEIGHT;
        for body in &self.bodies {
            let screen_pos = world_to_screen_coords(body.position, local_view_center, LOCAL_VIEW_HEIGHT, left_viewport);
            // Clipping check
            if left_viewport.contains(screen_pos) {
                let screen_radius = body.radius as f32 * local_scale; // Use local_scale
                let final_radius = screen_radius.max(1.0);

                let circle_mesh = graphics::Mesh::new_circle(ctx, graphics::DrawMode::fill(), screen_pos, final_radius, 0.1, body.color)?;
                canvas.draw(&circle_mesh, graphics::DrawParam::default());
            }
        }
        // Draw ship only if its center is within the viewport
        if left_viewport.contains(world_to_screen_coords(self.ship.position, local_view_center, LOCAL_VIEW_HEIGHT, left_viewport)) {
             self.draw_ship(&mut canvas, ctx, local_view_center, LOCAL_VIEW_HEIGHT, left_viewport, true)?;
        }

        // --- Draw Global View (Right Side) ---
        let global_view_center = Vec2::zeros(); // Center on Sun (0,0)
        // Use the current zoom level
        let current_global_view_height = GLOBAL_ZOOM_LEVELS[self.global_zoom_level];
        let global_scale = right_viewport.h / current_global_view_height;

        for body in &self.bodies {
            let screen_pos = world_to_screen_coords(body.position, global_view_center, current_global_view_height, right_viewport);
             // Clipping check
            if right_viewport.contains(screen_pos) {
                let screen_radius = body.radius as f32 * global_scale; // Use global_scale
                let final_radius = screen_radius.max(1.0);

                let circle_mesh = graphics::Mesh::new_circle(ctx, graphics::DrawMode::fill(), screen_pos, final_radius, 0.1, body.color)?;
                canvas.draw(&circle_mesh, graphics::DrawParam::default());
            }
        }
        // Draw ship only if its center is within the viewport
        if right_viewport.contains(world_to_screen_coords(self.ship.position, global_view_center, current_global_view_height, right_viewport)) {
            self.draw_ship(&mut canvas, ctx, global_view_center, current_global_view_height, right_viewport, false)?;
        }

        // --- Draw UI Overlays (Fuel, etc.) - Draw last, without viewport changes ---
        let fuel_text = format!("Fuel: {:.1}", self.ship.fuel);
        let text = graphics::Text::new(fuel_text);
        canvas.draw(&text, graphics::DrawParam::new().dest(Point2::new(10.0, 10.0)).color(Color::WHITE));

        // Draw viewport divider (optional)
        let divider = graphics::Mesh::new_line(
            ctx,
            &[Point2::new(SCREEN_WIDTH / 2.0, 0.0), Point2::new(SCREEN_WIDTH / 2.0, SCREEN_HEIGHT)],
            1.0, // Line width
            Color::from_rgb(50, 50, 50), // Dark gray
        )?;
        canvas.draw(&divider, graphics::DrawParam::default());

        canvas.finish(ctx)?;
        Ok(())
    }

    // --- Keyboard Event Handling ---
    fn key_down_event(
        &mut self,
        _ctx: &mut Context,
        input: KeyInput,
        _repeated: bool,
    ) -> GameResult {
        match input.keycode {
            Some(KeyCode::Left) => {
                self.is_rotating_left = true;
            }
            Some(KeyCode::Right) => {
                self.is_rotating_right = true;
            }
            Some(KeyCode::Up) => {
                self.is_thrusting = true;
            }
            Some(KeyCode::Minus) => {
                // Zoom out global view (increase index, clamp at max)
                self.global_zoom_level = (self.global_zoom_level + 1).min(GLOBAL_ZOOM_LEVELS.len() - 1);
            }
            Some(KeyCode::Equals) => {
                 // Zoom in global view (decrease index, clamp at min)
                self.global_zoom_level = self.global_zoom_level.saturating_sub(1);
            }
            _ => (), // Ignore other keys
        }
        Ok(())
    }

    fn key_up_event(&mut self, _ctx: &mut Context, input: KeyInput) -> GameResult {
        match input.keycode {
            Some(KeyCode::Left) => {
                self.is_rotating_left = false;
            }
            Some(KeyCode::Right) => {
                self.is_rotating_right = false;
            }
            Some(KeyCode::Up) => {
                self.is_thrusting = false;
            }
            _ => (), // Ignore other keys
        }
        Ok(())
    }
}

// Add helper method to GameState for drawing the ship (to avoid code duplication)
impl GameState {
    fn draw_ship(
        &self,
        canvas: &mut graphics::Canvas,
        ctx: &mut Context,
        view_center: Vec2,
        view_height: f32,
        viewport: graphics::Rect,
        detailed: bool, // Flag to draw triangle or just a dot
    ) -> GameResult {
        let ship_screen_pos = world_to_screen_coords(self.ship.position, view_center, view_height, viewport);

        if detailed {
            let ship_size: f32 = 10.0; // Increase fixed pixel size for local view ship
            let angle = self.ship.orientation as f32;
            let rot = glam::Mat2::from_angle(-angle);

            let p1 = rot * Point2::new(0.5 * ship_size, 0.0);
            let p2 = rot * Point2::new(-0.5 * ship_size, -0.3 * ship_size);
            let p3 = rot * Point2::new(-0.5 * ship_size, 0.3 * ship_size);

            let points = [
                ship_screen_pos + p1,
                ship_screen_pos + p2,
                ship_screen_pos + p3,
            ];
            let ship_mesh = graphics::Mesh::new_polygon(ctx, graphics::DrawMode::fill(), &points, Color::WHITE)?;
            canvas.draw(&ship_mesh, graphics::DrawParam::default());
        } else {
            // Draw simple dot for global view - make it bigger and green
            // Check clipping *before* drawing the dot
            if viewport.contains(ship_screen_pos) {
                let ship_dot = graphics::Mesh::new_circle(ctx, graphics::DrawMode::fill(), ship_screen_pos, 4.0, 0.2, Color::GREEN)?;
                canvas.draw(&ship_dot, graphics::DrawParam::default());
            }
        }
        Ok(())
    }
}

// --- Main Function ---
fn main() -> GameResult {
    let (ctx, event_loop) = ContextBuilder::new("orbit_rl_sim", "YourName")
        .window_setup(WindowSetup::default().title("Orbit RL - M1: Core Sim"))
        .window_mode(WindowMode::default().dimensions(SCREEN_WIDTH, SCREEN_HEIGHT))
        .build()?;

    let state = GameState::new()?;

    event::run(ctx, event_loop, state)
} 