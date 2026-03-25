mod lut_helper;

use lut_helper::{FingerLUT, FingerType};

fn main() {
    // Load the LUT file
    let lut_path = "../finger_tip_lut.npz";
    
    match FingerLUT::load(lut_path) {
        Ok(lut) => {
            println!("Successfully loaded LUT file!");
            
            // Get resolution
            println!("Resolution: {}", lut.get_resolution());
            
            // List available fingers
            println!("Available fingers: {:?}", lut.get_available_fingers());
            
            // Get a specific transform
            if let Some(transform) = lut.get_transform(FingerType::Index, 0) {
                println!("\nIndex finger transform at sample 0:");
                println!("{:.4}", transform.matrix);
            }
            
            // Interpolate between samples
            if let Some(transform) = lut.interpolate_transform(FingerType::Index, 0.5) {
                println!("\nIndex finger interpolated transform at t=0.5:");
                println!("{:.4}", transform.matrix);
            }
            
            // Combine thumb transforms
            let combined = lut.combine_thumb_transforms(5, 5);
            println!("\nCombined thumb transform (flex=5, opp=5):");
            println!("{:.4}", combined.matrix);
        }
        Err(e) => {
            eprintln!("Failed to load LUT file: {}", e);
        }
    }
}
