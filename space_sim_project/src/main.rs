use ggez::{Context, ContextBuilder, GameResult, GameError};
use ggez::conf::{WindowMode, WindowSetup};
use ggez::event::{self, EventHandler};
use ggez::graphics::{self, Color};
use ggez::glam;
use nalgebra::Vector2;

// Define a type alias for better readability
type Vec2 = Vector2<f64>;

// --- Simulation Constants ---
// Adjusted G for visualization - lower values prevent things from flying apart too quickly
// You might need to tune G, masses, and initial velocities together.
const G: f64 = 1.0;
// Use squared epsilon to avoid sqrt in check
const EPSILON_SQ: f64 = 1e-2; // Min squared distance for force calc (prevents extreme forces/division by zero)

// --- Graphics Constants ---
const SCREEN_WIDTH: f32 = 1000.0;
const SCREEN_HEIGHT: f32 = 800.0;
// How many pixels correspond to one unit in the simulation space. Adjust to zoom in/out.
const SIMULATION_SCALE: f32 = 4.0;


#[derive(Debug, Clone)]
struct Body {
    mass: f64,
    position: Vec2,
    velocity: Vec2,
}

impl Body {
    fn new(mass: f64, position: Vec2, velocity: Vec2) -> Self {
        Body { mass, position, velocity }
    }

    // Calculates the gravitational force exerted BY 'other' ON 'self'
    fn calculate_force(&self, other: &Body) -> Vec2 {
        let distance_vec = other.position - self.position;
        let distance_sq = distance_vec.norm_squared();

        // Check if bodies are too close
        if distance_sq < EPSILON_SQ {
             return Vec2::zeros(); // Return zero force if too close
        }

        // No need for sqrt if using distance_sq in the formula
        let force_magnitude = G * self.mass * other.mass / distance_sq;

        // Calculate direction vector (normalized distance vector)
        // Avoids a sqrt compared to distance_vec.normalize() if distance_sq is already known
        let force_direction = distance_vec / distance_sq.sqrt();

        force_direction * force_magnitude
    }

    fn update_velocity(&mut self, total_force: Vec2, dt: f64) {
        // Check for zero mass to prevent division by zero if implementing massless particles later
        if self.mass == 0.0 { return; }
        let acceleration = total_force / self.mass;
        self.velocity += acceleration * dt;
    }

    fn update_position(&mut self, dt: f64) {
        self.position += self.velocity * dt;
    }
}

// --- Game State ---
struct GameState {
    bodies: Vec<Body>,
    dt: f64, // Simulation time step
}

impl GameState {
    // Creates the initial state of the simulation
    fn new() -> GameResult<GameState> {
        // Adjust masses, positions, and velocities for a more visually stable simulation
        let bodies = vec![
            // Central "Star" - more massive, initially stationary
            Body::new(10000.0, Vec2::new(0.0, 0.0), Vec2::new(0.0, 0.0)),
            // Planet 1 - moderate mass, moderate distance, moderate velocity
            Body::new(10.0, Vec2::new(100.0, 0.0), Vec2::new(0.0, 15.0)),
            // Planet 2 - smaller mass, different position/velocity
            Body::new(5.0, Vec2::new(-50.0, -80.0), Vec2::new(10.0, -10.0)),
             // Moon for Planet 1? - small mass, close to planet 1, velocity relative to planet 1
            Body::new(0.1, Vec2::new(110.0, 0.0), Vec2::new(0.0, 15.0 + 5.0)),
        ];
        // Smaller time step often needed for stability in visualization
        let dt = 0.01;
        Ok(GameState { bodies, dt })
    }

    // Helper function to convert simulation coordinates to screen coordinates
    fn world_to_screen_coords(&self, world_pos: Vec2) -> glam::Vec2 {
        // glam::Vec2 is used by ggez for drawing
        glam::Vec2::new(
            // Scale the position
            (world_pos.x as f32 * SIMULATION_SCALE)
            // Offset to center the view
            + SCREEN_WIDTH / 2.0,
            (world_pos.y as f32 * SIMULATION_SCALE) + SCREEN_HEIGHT / 2.0,
        )
    }
}

// Implement ggez's EventHandler trait for our GameState
impl EventHandler<GameError> for GameState {

    // Update the simulation state (called before drawing each frame)
    fn update(&mut self, _ctx: &mut Context) -> GameResult {
        let num_bodies = self.bodies.len();
        // Store forces calculated in this time step
        let mut forces = vec![Vec2::zeros(); num_bodies];

        // --- Calculate Forces ---
        // Iterate through all unique pairs of bodies (i, j)
        for i in 0..num_bodies {
            for j in (i + 1)..num_bodies { // Optimization: only calculate each pair once
                let force = self.bodies[i].calculate_force(&self.bodies[j]);
                forces[i] += force;      // Force on body i due to body j
                forces[j] -= force;      // Force on body j due to body i (Newton's 3rd Law)
            }
        }

        // --- Update Velocities and Positions ---
        for i in 0..num_bodies {
            self.bodies[i].update_velocity(forces[i], self.dt);
            self.bodies[i].update_position(self.dt);
        }

        Ok(()) // Indicate success
    }

    // Draw the simulation state to the screen (called after update)
    fn draw(&mut self, ctx: &mut Context) -> GameResult {
        // Start drawing by creating a Canvas
        let mut canvas = graphics::Canvas::from_frame(ctx, graphics::Color::BLACK);

        // Draw each body
        for body in &self.bodies {
            // Calculate size based on mass (log scale helps visualize large mass differences)
            // Clamp minimum size for visibility
            let radius = (body.mass.max(1.0).log10().max(0.5) * 2.0 + 1.0) as f32;
            // Convert simulation position to screen position
            let screen_pos = self.world_to_screen_coords(body.position);

            // Create a white circle mesh for the body
            let circle_mesh = graphics::Mesh::new_circle(
                ctx,                // The ggez context
                graphics::DrawMode::fill(), // Fill the circle
                screen_pos,         // Center position on screen
                radius,             // Radius on screen
                0.1,                // Tolerance (lower is smoother)
                Color::WHITE,       // Color
            )?; // The '?' handles potential errors

            // Draw the mesh to the canvas
            canvas.draw(&circle_mesh, graphics::DrawParam::default());
        }

        // Finish drawing and present the frame
        canvas.finish(ctx)?; // The '?' handles potential errors

        Ok(()) // Indicate success
    }
}

// --- Main Function ---
fn main() -> GameResult {
    // Create a context and event loop using the ContextBuilder
    let (ctx, event_loop) = ContextBuilder::new("n_body_sim", "Cool Simulation Inc")
        // Configure the window
        .window_setup(WindowSetup::default().title("N-Body Simulation!"))
        .window_mode(WindowMode::default().dimensions(SCREEN_WIDTH, SCREEN_HEIGHT))
        // Build the context
        .build()?; // The '?' handles potential errors

    // Create the initial state of our game/simulation
    let state = GameState::new()?; // The '?' handles potential errors

    // Start the ggez event loop!
    event::run(ctx, event_loop, state)
}

// --- Minor Optimization in force calculation ---
// The previous nested loop calculated force(i, j) and force(j, i) separately.
// Since force(j, i) = -force(i, j) (Newton's 3rd Law), we can optimize:
//
// for i in 0..num_bodies {
//     for j in (i + 1)..num_bodies { // Note: j starts from i + 1
//         let force = self.bodies[i].calculate_force(&self.bodies[j]);
//         forces[i] += force;
//         forces[j] -= force; // Apply the equal and opposite force
//     }
// }
// This halves the number of calls to calculate_force. I've included this optimization
// in the code above.
