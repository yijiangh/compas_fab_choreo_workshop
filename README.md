<!-- 
**Quick links:** [compas docs](https://compas-dev.github.io/main/) | [compas_assembly docs](https://blockresearchgroup.github.io/compas_assembly/) | [compas_fab docs](https://gramaziokohler.github.io/compas_fab/latest/) | [slides](slides.pdf) | [system overview](#system-overview) | [troubleshooting](#troubleshooting)

**Progress:** [0. requirements](#requirements) | [1. installation](#getting-started) | [2. editor setup](#setting-up-your-development-environment) | [3. create assembly](#create-an-assembly---brick-wall) | [4. robotic planning](#planning-robotic-fabrication-of-assembly) | [5. robotic execution](#executing-robotic-fabrication) -->

# Workshop: Sequence and Motion Planning for Robotic Assembly with Choreo and COMPAS framework

[![Build Status](https://travis-ci.com/yijiangh/compas_fab_choreo_workshop.svg?branch=master)](https://travis-ci.com/yijiangh/compas_fab_choreo_workshop)
[![GitHub - License](https://img.shields.io/github/license/compas-dev/compas.svg)](./LICENSE)

Materials for the Sequence and Motion Planning for Robotic Spatial Assembly workshop using COMPAS framework and Choreo as the planning engine.

> During this workshop, we will ...

## Requirements

* Operating System: **Windows 10** Pro or better <sup>(1)</sup>.
* [Rhinoceros 3D 6.0](https://www.rhino3d.com/): Focus on Rhino 6.0 only. [See here if you use Rhino 5.0](#rhino-50)
* [Anaconda Python Distribution](https://www.anaconda.com/download/): 2.7 or 3.x
* [Docker Community Edition](https://www.docker.com/get-started): Download it for [Windows](https://store.docker.com/editions/community/docker-ce-desktop-windows) or [Mac](https://store.docker.com/editions/community/docker-ce-desktop-mac).
* Git: [official command-line client](https://git-scm.com/) or visual GUI (e.g. [Github Desktop](https://desktop.github.com/) or [SourceTree](https://www.sourcetreeapp.com/))

<!-- > Note: if you get an error, scroll down to the [Troubleshooting](#troubleshooting) section. -->
<sup>(1): Windows 10 Home does not support running Docker.</sup>

<details><summary>Rhino 5.0</summary>
The focus of the workshop will be on Rhino 6.0 only. While most things will work on Rhino 5.0, it is not recommended as there are several manual steps required to get the software to run.

However, if you do use Rhino 5.0, make sure to install the following:
​
* [Grasshopper](https://www.grasshopper3d.com/)
* [GHPython](https://www.food4rhino.com/app/ghpython)
* [IronPython 2.7.5](https://github.com/IronLanguages/main/releases/tag/ipy-2.7.5) ([see here for details about this manual update](https://compas-dev.github.io/main/environments/rhino.html#ironpython-1)).
</details>


## Getting started

<!-- ![Progress](images/progress-1.png) -->

We will install all the required **COMPAS** packages using Anaconda. Anaconda uses **environments** to create isolated spaces for projects' depedencies, it is recommendable that you do all the exercises in a newly created environment.

First, clone this repository. You have two options:

<details><summary>1. Using a visual client <i>(e.g. SourceTree)</i></summary>
Open your GIT visual client (e.g. SourceTree), and clone the repository (on SourceTree, `File -> Clone / New`) and enter the following URL and the destination folder:

      https://github.com/yijiangh/compas_fab_choreo_workshop.git

</details>

<details><summary>2. Using git command line client</summary>
Start your Anaconda Prompt, go to the destination folder where you wish to place all the workshop material and run:

      git clone https://github.com/yijiangh/compas_fab_choreo_workshop.git

</details>

<br/>

Now, we can create the environment and install all packages. Start your Anaconda Prompt, go to the repository folder you just cloned, and run:

      conda env create -f workshop.yml -n workshop
      conda activate workshop

<details><summary>Not working?</summary>

Make sure you really changed into the repository folder. For example, if you cloned the repository into a folder called `Code` in your home directory, you should type:

**On Mac**

      cd ~/Code/compas_fab_choreo_workshop

**On Windows**

      cd %USERPROFILE%\Code\compas_fab_choreo_workshop

If the command fails because you already have an environment with the same name, choose a different one, or remove the old one before creating the new one:

      conda remove -n workshop --all

</details>

<br/>

Great! Now type `python` in your Anaconda Prompt, and test if the installation went well:

      >>> import compas
      >>> import compas_fab

If that doesn't fail, you're good to go! Exit the python interpreter (either typing `exit()` or pressing `CTRL+Z` followed by `Enter`).

Now let's make all the installed packages available inside Rhino. Still from the Anaconda Prompt, type the following:

      python -m compas_rhino.install -v 6.0 -p compas compas_ghpython compas_rhino compas_fab roslibpy

Congrats! 🎉 You are all set! Open Rhino and try to import `compas` to verify everything is working fine.

<!-- ## Setting up your development environment -->
---

## Exercises

### Create an assembly planning instance

TODO: clone https://github.com/yijiangh/compas_fab.assembly_tests 

<!-- ---

## System overview

Environments? Containers? Processes? Confused? 😵 What is connected to what and how?

The following diagram shows how the different parts are interconnected and which one calls which other:

![System overview](images/overview.png)

---

## Troubleshooting -->
