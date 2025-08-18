# User Guide — crazyflie-GUI-python

This guide explains how to download and update the `crazyflie-GUI-python` repository using Git.

## 1. Prerequisites

- [Git](https://git-scm.com/downloads) installed
- [Python 3](https://www.python.org/downloads/) installed

## 2. First-time Download (Clone the Repository)

Follow these steps to get the project onto your machine:

1. Open a terminal.
2. Clone the repository:
   ```bash
   git clone https://github.com/Lee-Chun-Yi/crazyflie-GUI-python.git
   ```
3. Move into the project directory:
   ```bash
   cd crazyflie-GUI-python
   ```
4. Install setup tools & the project:
   ```bash
   python -m pip install -U pip setuptools wheel
   python -m pip install -e .
   ```
5. Start the application:
   ```bash
   cfmarslab
   ```

## 3. Updating to the Latest Version

To update an existing local copy:

1. Open a terminal and change into the project directory:
   ```bash
   cd crazyflie-GUI-python
   ```
2. Fetch and merge the latest changes:
   ```bash
   git pull origin main
   ```
