# Installation

Prerequisites: Install [Poetry](#poetry)

Clone this repository and go to its root:

    git clone git@github.com:tud-airlab/base_mpc.git
    cd base-mpc
    
Install dependencies

    poetry install
    
Get ForcesPro license (see [ForcesPro installation](#forcespro))





## Other Dependencies
Below you find instructions for installing dependencies (optionally) required.

### Poetry 
Poetry is a python packaging and dependency management tool. It allows you to declare the libraries your project depends on and it will manage (install/update) them for you. We advice you to use this tool to install all required dependencies. However it is not a requirement and a simple pip install for all depedencies will work as well. 

For more information we refer to the [poetry documentation](https://python-poetry.org/docs/).

### ForcesPro

- Obtain a license from the [Embotech site](https://my.embotech.com/auth/sign_up) (this takes about half a week)
- When your license is approved, assign your license to your computer. Make sure to use the username of your account.
- Download the fingerprinter and run it on your system. Paste the fingerprint into your forces license.
- Download the forces client and extract the files to (e.g., <your_ros_workspace>/src/base_mpc/python_solver_generation/forces).
