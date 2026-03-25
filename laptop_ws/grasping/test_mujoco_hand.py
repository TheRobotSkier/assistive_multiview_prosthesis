#!/usr/bin/env python3
"""
Simple script to load the MIA hand in MuJoCo and open the normal GUI viewer. 
"""

import mujoco as mj
import os

# Get the directory where this script is located
script_dir = os.path.dirname(os.path.abspath(__file__))

# Path to the MIA hand XML file (scene with floor and proper lighting)
xml_path = os.path.join(script_dir, 'mia_hand_mujoco', 'mia_hand', 'scene_left.xml')

print(f"Loading MIA hand from: {xml_path}")

# Load the XML file
spec = mj.MjSpec.from_file(xml_path)

# Compile the model
model = spec.compile()

# Create data structure
data = mj.MjData(model)

# Reset the simulation
mj.mj_resetData(model, data)

print("Model loaded successfully!")
print(f"Number of bodies: {model.nbody}")
print(f"Number of joints: {model.njnt}")
print(f"Number of actuators: {model.nu}")

# Diagnostic: Check for lights in the model
print(f"\n=== DIAGNOSTIC INFO ===")
print(f"Number of lights in model: {model.nlight}")
if model.nlight == 0:
    print("WARNING: No lights found in the model! This will cause a black screen.")

# Diagnostic: Check model statistics
print(f"Model stat center: {model.stat.center}")
print(f"Model stat extent: {model.stat.extent}")

# Diagnostic: Check first body position (palm_l)
if model.nbody > 0:
    print(f"First body position: {data.xpos[0]}")

# Diagnostic: Check if geoms are loaded
print(f"Number of geoms: {model.ngeom}")
print(f"Number of meshes: {model.nmesh}")
print(f"========================\n")

# Initialize GLFW
if not mj.glfw.glfw.init():
    raise RuntimeError("Failed to initialize GLFW")

# Create a windowed mode window and its OpenGL context
window = mj.glfw.glfw.create_window(800, 600, "MIA Hand - MuJoCo Viewer", None, None)
if not window:
    mj.glfw.glfw.terminate()
    raise RuntimeError("Failed to create GLFW window")

# Make the window's context current
mj.glfw.glfw.make_context_current(window)

# Create the camera
cam = mj.MjvCamera()
mj.mjv_defaultCamera(cam)
cam.distance = 0.5
cam.elevation = -30
cam.azimuth = 90

# FIX: Set camera lookat to the scene's center position
# Based on scene_left.xml statistic center="0.1 0 0.1"
cam.lookat[0] = 0.1
cam.lookat[1] = 0.0
cam.lookat[2] = 0.1

# Diagnostic: Check camera lookat (default is 0,0,0)
print(f"\n=== CAMERA INFO ===")
print(f"Camera lookat: {cam.lookat}")
print(f"Camera distance: {cam.distance}")
print(f"Camera elevation: {cam.elevation}")
print(f"Camera azimuth: {cam.azimuth}")
print(f"==================\n")

# Create the scene with enough space for all geoms
scene = mj.MjvScene(model, maxgeom=10000)

# Create the visualization options
opt = mj.MjvOption()
mj.mjv_defaultOption(opt)

# Create the rendering context
ctx = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150)

# Create the rendering viewport
viewport = mj.MjrRect(0, 0, 800, 600)

print("Starting MuJoCo viewer...")
print("Close the window to exit.")

try:
    # Main simulation loop
    while not mj.glfw.glfw.window_should_close(window):
        # Step the physics
        mj.mj_step(model, data)
        
        # Update the scene
        mj.mjv_updateScene(model, data, opt, None, cam, mj.mjtCatBit.mjCAT_ALL, scene)
        
        # Render the scene
        mj.mjr_render(viewport, scene, ctx)
        
        # Swap buffers and poll events
        mj.glfw.glfw.swap_buffers(window)
        mj.glfw.glfw.poll_events()

except KeyboardInterrupt:
    print("\nShutting down...")
finally:
    # Cleanup
    mj.glfw.glfw.destroy_window(window)
    mj.glfw.glfw.terminate()
    print("Viewer closed.")
