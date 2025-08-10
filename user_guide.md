# User Guide — crazyflie-GUI-python

This guide explains how to **download** and **update** the `crazyflie-GUI-python` repository using the command line (Git).

---

## 1. Prerequisites

- **Git** installed on your system  
  - [Download Git](https://git-scm.com/downloads)

---

## 2. First-time Download (Clone the Repository)

Open your terminal and run:

```bash

git clone https://github.com/Lee-Chun-Yi/crazyflie-GUI-python.git

````

Once the file is downloaded, enter: 

```bash
cfmarslab
``` 

in terminal to launch

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

**Repository Link:** [https://github.com/Lee-Chun-Yi/crazyflie-GUI-python](https://github.com/Lee-Chun-Yi/crazyflie-GUI-python)



