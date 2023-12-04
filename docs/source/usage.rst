Usage
=====

.. _installation:

Installation
------------

1. Clone the repository to your local machine using the follwing command:

.. code-block::

    git clone https://github.com/Abenezergirma/Chapter-2.git


2. Ensure you have the required softwares installed with their dependancies (MATLAB, Python, Simulink, etc.).
3. Set up the project environment following the guidelines in the project documentation.

Project Guideline
-----------------

Follow these steps to effectively utilize the project:

1. **Trajectory Planning and Energy Analysis**:
   Start the project by executing :code:`main.m`. This script is responsible for performing trajectory planning and analyzing the energy requirements of each aircraft.

2. **Energy Requirement Simulation**:
   Use :code:`generateEnergyRequirement.m` to simulate the aircraft's operation and determine its energy requirements. This step is crucial for understanding the power consumption patterns under various conditions.

3. **Battery Prognostics Analysis**:
   Dive into battery state of charge (SoC) predictions by utilizing the Python library found in the :code:`Battery Prognostics` folder. This analysis helps in assessing the longevity and efficiency of the UAV's battery.

4. **Development of Reward Functions**:
   Go to :code:`RewardFunctionExperiments` to develop and experiment with various reward functions. These functions are essential for evaluating different operational strategies based on their outcomes.

5. **Decision Making Based on Risk Assessment**:
   Finally, execute :code: `decision_maker.py`to apply the decision-making regarding whether to execute the flight or not. This script integrates the risk assessment and makes informed decisions to optimize UAV operations.

For more detailed information on each step, refer to the corresponding sections in the documentation.


