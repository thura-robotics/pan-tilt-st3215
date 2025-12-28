# pt_camera Virtual Environment Setup

This guide provides instructions on how to create a virtual environment named `pt_camera` and install the necessary dependencies from `requirements.txt`.

## Prerequisites

- Python 3 installed on your system.
- `requirements.txt` file present in the current directory.

## Steps

### 1. Create the Virtual Environment

Run the following command to create a virtual environment named `pt_camera`:

```bash
python3 -m venv pt_camera
```

### 2. Activate the Virtual Environment

Activate the virtual environment using the command appropriate for your shell:

**Bash/Zsh:**
```bash
source pt_camera/bin/activate
```

**Fish:**
```fish
source pt_camera/bin/activate.fish
```

**Windows (Command Prompt):**
```cmd
pt_camera\Scripts\activate
```

**Windows (PowerShell):**
```powershell
.\pt_camera\Scripts\Activate.ps1
```

### 3. Install Dependencies

Once the virtual environment is activated, upgrade `pip` and install the dependencies from `requirements.txt`:

```bash
pip install --upgrade pip
pip install -r requirements.txt
```

> **Note:** Some packages in `requirements.txt` (e.g., `tensorrt`, `Jetson.GPIO`, ROS 2 packages) may rely on system-level libraries or might properly be installed via the system package manager (apt) on Jetson platforms. If you encounter errors, you might need to enable access to system site packages or install specific libraries globally.
>
> To create a venv with access to system packages, you can use:
> ```bash
> python3 -m venv --system-site-packages pt_camera
> ```

### 4. Deactivate

To exit the virtual environment when you are done, simply run:

```bash
deactivate
```
