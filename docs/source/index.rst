Pre‑departure Flight Planning and Risk Assessment for UAM/UAS Operations under Battery Constraint
===================================
Contents
--------

.. toctree::

   usage
   api


Overview
--------

The goal of this project is to develop a decision-making framework for Urban Air Mobility (UAM) and Unmanned Aerial Systems (UAS) operations, focusing on enhancing collision safety and battery energy use. The framework features a novel two-layer algorithm: the upper layer performs strategic de-confliction, and the lower layer handles prognostics and decision-making for mission execution. The following figure shows the overall structure and working principle of the developed framework.

    
.. image:: images/framework.png
   :alt: alternate text
   :align: center
   :width: 70%

    
    
Package Delivery Scenario   
--------
The scenario we used to implement the framework and test its performance is designed for a package delivery application and is implemented for the University Park area of the Dallas-Fort Worth metropolitan region. It includes three depots from which aircraft are expected to take off, and random destination places can be assigned within the $7 km^2$ rectangular area.

.. image:: images/PackageDeliveryScenario.jpg
   :alt: alternate text
   :align: center

   
Results
--------
.. table::
   :align: center

   +----------------------------------------+----------------------------------------+
   | .. image:: images/SOC_1.png            | .. image:: images/SOC_2.png            |
   |    :alt: alternative text 1            |    :alt: alternative text 2            |
   |    :align: center                      |    :align: center                      |
   +----------------------------------------+----------------------------------------+
   | .. image:: images/SOC_3.png            | .. image:: images/SOC_4.png            |
   |    :alt: alternative text 3            |    :alt: alternative text 4            |
   |    :align: center                      |    :align: center                      |
   +----------------------------------------+----------------------------------------+
   | .. image:: images/SOC_5.png            | .. image:: images/SOC_6.png            |
   |    :alt: alternative text 5            |    :alt: alternative text 6            |
   |    :align: center                      |    :align: center                      |
   +----------------------------------------+----------------------------------------+



Check out the :doc:`usage` section for further information, including
how to :ref:`installation` the project.

.. note::

   This project is under active development.

