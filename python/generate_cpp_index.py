import os

SITE_CPP_DIR = "site/myProjectCpp"
OUTPUT_INDEX = "docs/myProjectCpp/index.md"  # Adjust this as needed

def generate_html_index():
    if not os.path.exists(SITE_CPP_DIR):
        print(f"[ERROR] Cannot find {SITE_CPP_DIR}. Run `mkdocs build` first.")
        return

    entries = [
        d for d in os.listdir(SITE_CPP_DIR)
        if os.path.isdir(os.path.join(SITE_CPP_DIR, d)) and
        os.path.exists(os.path.join(SITE_CPP_DIR, d, "index.html"))
    ]

    if not entries:
        print("[WARNING] No index.html entries found.")
        return

    os.makedirs(os.path.dirname(OUTPUT_INDEX), exist_ok=True)

    with open(OUTPUT_INDEX, "w") as f:
        f.write("# C++ API Documentation\n\n")
        for entry in sorted(entries):
            label = entry.replace("class_", "Class: ").replace("_", " ").title()
            f.write(f"- [{label}](../myProjectCpp/{entry}/)\n")

    print(f"[OK] Wrote generated index: {OUTPUT_INDEX}")

if __name__ == "__main__":
    generate_html_index()
