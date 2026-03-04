import sys
import os
import hashlib
import urllib.request
import json
import re
import ssl

def calculate_sha512(file_path):
    sha512_hash = hashlib.sha512()
    with open(file_path, "rb") as f:
        for byte_block in iter(lambda: f.read(4096), b""):
            sha512_hash.update(byte_block)
    return sha512_hash.hexdigest().lower()

def update_port(version):
    repo = "Olli1080/CoordTrafoUtil"
    url = f"https://github.com/{repo}/archive/refs/tags/v{version}.tar.gz"
    temp_file = "temp_archive.tar.gz"

    print(f"Downloading archive from {url}...")
    try:
        # Create unverified context to avoid SSL certificate errors on some systems
        context = ssl._create_unverified_context()
        with urllib.request.urlopen(url, context=context) as response, open(temp_file, 'wb') as out_file:
            out_file.write(response.read())
    except Exception as e:
        print(f"Error: Failed to download archive. Ensure the version v{version} exists on GitHub.")
        print(e)
        sys.exit(1)

    print("Calculating SHA512 hash...")
    hash_val = calculate_sha512(temp_file)
    os.remove(temp_file)

    portfile_path = "vcpkg-port/portfile.cmake"
    vcpkg_json_path = "vcpkg-port/vcpkg.json"

    print(f"Updating {portfile_path}...")
    with open(portfile_path, "r") as f:
        content = f.read()

    # Update REF and SHA512
    content = re.sub(r'REF v[0-9.]+', f'REF v{version}', content)
    content = re.sub(r'REF "v\$\{VERSION\}"', f'REF v{version}', content)
    content = re.sub(r'SHA512 [a-f0-9]+', f'SHA512 {hash_val}', content)
    content = re.sub(r'SHA512 # TODO:.*', f'SHA512 {hash_val}', content)

    with open(portfile_path, "w") as f:
        f.write(content)

    print(f"Updating version in {vcpkg_json_path}...")
    with open(vcpkg_json_path, "r") as f:
        data = json.load(f)
    
    data["version"] = version
    
    with open(vcpkg_json_path, "w") as f:
        json.dump(data, f, indent=2)

    print(f"\nSuccessfully updated vcpkg port to version {version}!")
    print(f"Hash: {hash_val}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python update_vcpkg_port.py <version>")
        sys.exit(1)
    
    update_port(sys.argv[1])
