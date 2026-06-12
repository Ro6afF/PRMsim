# 🤖 Interactive PRM Motion Planning Simulation

An interactive 3D web application for visualizing and experimenting with Probabilistic Roadmap (PRM) algorithms for robot manipulators, developed as part of a master's dissertation.

This simulation allows users to construct custom robotic arms using Denavit-Hartenberg (DH) parameters, place obstacles in a 3D environment, and evaluate different sampling and path-finding strategies. To handle computationally heavy tasks like collision detection and graph search, the core algorithms are written in Rust and compiled to WebAssembly (WASM).

## ✨ Features

- **Interactive 3D Environment**: Built with React Three Fiber, allowing real-time visualization of the robot arm, configuration samples, the generated roadmap, and the final planned path.
- **Customizable Kinematics**: Define and tweak the robot's structure on the fly using Denavit-Hartenberg (DH) parameters.
- **Dynamic Obstacles**: Add, move, and resize spherical obstacles within the workspace to test collision avoidance.
- **Advanced PRM Tools**: 
  - Compare multiple sampling strategies (Uniform, Gaussian, Bridge).
  - Configure roadmap connection heuristics (e.g., K-Nearest neighbors, distance thresholds).
- **High-Performance Compute**: Forward kinematics, collision detection, and pathfinding are offloaded to a Rust/WASM backend for maximum efficiency.
- **State Persistence**: Simulation state is automatically compressed and saved locally so you don't lose your setup on reload.

## 🛠️ Tech Stack

- **Frontend**: React 19, TypeScript, Vite, Bootstrap 5
- **3D Visualization**: Three.js, React Three Fiber (`@react-three/fiber`), Drei
- **Compute Backend**: Rust, WebAssembly (`wasm-pack`)

## 🚀 Getting Started

### Prerequisites

To run this project locally, you will need:
- [Node.js](https://nodejs.org/) (v18 or newer recommended)
- [Rust](https://www.rust-lang.org/tools/install)
- [wasm-pack](https://rustwasm.github.io/wasm-pack/) (for building the WebAssembly module)

### Installation & Running

1. **Clone the repository and navigate to the project directory:**
   ```bash
   git clone <repository-url>
   cd PRMsim
   ```

2. **Install frontend dependencies:**
   ```bash
   npm install
   ```

3. **Start the development server:**
   ```bash
   npm run dev
   ```
   *Note: This command is configured to automatically build the Rust WASM module (`npm run build:wasm`) before starting the Vite development server.*

4. **Open your browser:** Navigate to `http://localhost:5173` to view the simulation.

## 📊 CLI Benchmarking

In addition to the interactive web interface, this project includes a powerful Rust-based Command Line Interface (CLI) for running large-scale automated benchmark sweeps of different PRM configurations. 

The benchmarking tool allows you to evaluate multiple sampling strategies, connection heuristics, and pathfinding settings across thousands of iterations using parallel processing, and it outputs the results to a CSV file for statistical analysis.

For instructions on how to configure and run the CLI benchmark, see the [prm-compute README](src/prm-compute/README.md).

## 📁 Project Structure

- `src/components/`: React components for UI panels, controls, and 3D scene elements.
- `src/prm-compute/`: The Rust backend containing the logic for PRM sampling, graph building, kinematics, pathfinding, and the CLI benchmark tool.
- `src/wasm/`: The destination directory where the compiled WASM package is outputted.
- `src/utils/`: Helper functions (e.g., state compression/decompression).
