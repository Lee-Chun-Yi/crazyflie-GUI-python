# User Guide — crazyflie-GUI-python

This guide explains how to **download** and **update** the `crazyflie-GUI-python` repository using the command line (Git).

---

## 1. Prerequisites

- **Git** installed on your system  
  - [Download Git](https://git-scm.com/downloads)
- A terminal environment:
  - **Windows**: PowerShell / Command Prompt
  - **macOS / Linux**: Terminal

---

## 2. First-time Download (Clone the Repository)

Open your terminal and run:

```bash

git clone https://github.com/Lee-Chun-Yi/crazyflie-GUI-python.git
cd crazyflie-GUI-python

````

once the file is downloaded, enter: ```cfmarslab``` in terminal to open it 

---

## 3. Updating to the Latest Version

When a new update is available, run:

```bash
cd crazyflie-GUI-python
git pull --rebase --autostash
```

This will:

* Download the latest changes from GitHub
* Preserve your local modifications by stashing and reapplying them

---

## 4. Common Issues

* **Pull conflicts due to local changes**
  To discard your local modifications:

  ```bash
  git restore --staged -W .
  git pull --rebase
  ```

* **Permission denied (SSH)**
  Ensure you have added your SSH key to GitHub and are using the SSH clone URL.

* **No Git installed**
  Install Git from [https://git-scm.com/downloads](https://git-scm.com/downloads).

---

## 5. Optional — Running the Project

If the project contains Python code, you can set up a virtual environment and install dependencies:

```bash
cd crazyflie-GUI-python

# Create virtual environment
python -m venv .venv

# Activate it
# Windows
.\.venv\Scripts\Activate.ps1
# macOS / Linux
source .venv/bin/activate

# Install dependencies
pip install -U pip
pip install -r requirements.txt
```

Run the project (replace with actual entry point):

```bash
python -m src.main
```

---

**Repository Link:** [https://github.com/Lee-Chun-Yi/crazyflie-GUI-python](https://github.com/Lee-Chun-Yi/crazyflie-GUI-python)



