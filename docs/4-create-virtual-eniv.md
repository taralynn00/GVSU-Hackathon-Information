4. [Creating a Virtual Environment](#create-virtual-environment)  



# Create Virtual Environment:
A virtual environment (venv) is essentially a clean, isolated workspace for your Python project.

Imagine you have two different projects:
- Project A needs an older version of a specific package (a tool).
- Project B needs the newest version of that same package.

If you installed all packages in the same place, installing the new version for Project B would break Project A.

The venv solves this by creating a separate, private box in your computer for each project. Consider your computer as a big box with mutiple smaller boxes inside of it. When you activate a venv, you ensure that any packages you install only exist inside that box, preventing any errors or side effects with your other projects (boxes) or your main computer setup.

`python -m venv testVenv`
*It will create this inside of whatever folder you’re in.*
*"testVenv" can be replaced with whatever you want to name your virtual environment.*

## Start Virtual Environment:
This is where you want to create that box so you don't mess with other projects. Using 'venv' in the commmand is calling the specific Python module that handles creating virtual environments.
`source /home/techshow1/Documents/TestCode/testVenv/bin/activate`

## Deactivate Virtual Environment:
To deactivate and leave the venv, you must already be inside of the venv. Navigate back to the venv folder and run this command:
`deactivate`

## Where to create python files in a venv:
You typically put them inside of the venv folder itself. In this example: "/home/techshow1/Documents/TestCode/testVenv”
