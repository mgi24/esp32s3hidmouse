import os

def update_file(path):
    if not os.path.exists(path):
        print(f"Path {path} does not exist")
        return
    with open(path, 'r') as f:
        content = f.read()
    
    new_content = content.replace("CONFIG_TINYUSB_HID_COUNT=1", "CONFIG_TINYUSB_HID_COUNT=2")
    if new_content != content:
        with open(path, 'w') as f:
            f.write(new_content)
        print(f"Updated {path}")
    else:
        print(f"No changes for {path} (already updated or pattern not found)")

update_file("c:/Users/mishb/Documents/tusb_hid/sdkconfig")
update_file("c:/Users/mishb/Documents/tusb_hid/sdkconfig.defaults")
