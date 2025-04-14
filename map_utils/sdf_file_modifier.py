import re
import sys
from pathlib import Path

def modify_sdf_file(file_path):
    """
    Modifies an SDF file by:
    1. Adding XML declaration at the top
    2. Replacing ambient tags with expanded material properties
    3. Adding material properties to ground_plane model
    4. Changing the world name to match the new filename
    5. Saving to a new file with '_1' appended to the original name
    
    Args:
        file_path (str): Path to the SDF file to be modified
    
    Returns:
        str: Path to the new file
    """
    # Read the original file
    with open(file_path, 'r') as f:
        content = f.read()
    
    # Create new filename with '_1' appended
    file_path = Path(file_path)
    new_stem = f"{file_path.stem}_1"
    new_file_path = file_path.with_stem(new_stem)
    
    # Add XML declaration if it doesn't exist
    if not content.startswith('<?xml'):
        content = '<?xml version=\'1.0\' encoding=\'ASCII\'?>\n' + content
    
    # Replace world name='default' with the new filename (without extension)
    world_pattern = r'<world name=\'default\'>'
    world_replacement = f"<world name='{new_stem}'>"
    content = re.sub(world_pattern, world_replacement, content)
    
    # Replace ambient tags in visual elements
    pattern = r'(<visual[^>]*>.*?)<ambient>1 1 1 1</ambient>(.*?</visual>)'
    replacement = r'\1<ambient>0.9 0.9 0.9 1</ambient>\n' \
                 r'            <diffuse>0.9 0.9 0.9 1</diffuse>\n' \
                 r'            <specular>0.9 0.9 0.9 1</specular>\n' \
                 r'            <emissive>0.4 0.4 0.4 1</emissive>\2'
    
    # Use re.DOTALL to make '.' match newlines as well
    modified_content = re.sub(pattern, replacement, content, flags=re.DOTALL)
    
    # Add material properties after </script> in ground_plane model's material
    ground_plane_pattern = r'(model name=\'ground_plane\'.*?<material>.*?<script>.*?</script>)(.*?</material>)'
    ground_plane_replacement = r'\1\n              <ambient>0.9 0.9 0.9 1</ambient>\n' \
                              r'              <diffuse>0.9 0.9 0.9 1</diffuse>\n' \
                              r'              <specular>0.9 0.9 0.9 1</specular>\n' \
                              r'              <emissive>0.4 0.4 0.4 1</emissive>\2'
    
    modified_content = re.sub(ground_plane_pattern, ground_plane_replacement, modified_content, flags=re.DOTALL)
    
    # Write to the new file
    with open(new_file_path, 'w') as f:
        f.write(modified_content)
    
    return str(new_file_path)

def process_files(files):
    """
    Process multiple SDF files
    
    Args:
        files (list): List of file paths to process
    """
    for file_path in files:
        if not file_path.endswith('.sdf'):
            print(f"Skipping {file_path} - not an SDF file")
            continue
        
        try:
            new_file = modify_sdf_file(file_path)
            print(f"Successfully processed {file_path} -> {new_file}")
        except Exception as e:
            print(f"Error processing {file_path}: {str(e)}")

if __name__ == "__main__":
    # Get files from command line arguments
    if len(sys.argv) < 2:
        print("Usage: python sdf_modifier.py file1.sdf [file2.sdf ...]")
        print("Or use wildcard: python sdf_modifier.py *.sdf")
        sys.exit(1)
    
    process_files(sys.argv[1:])