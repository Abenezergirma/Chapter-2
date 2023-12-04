Results
=========

After generating the trajectories using the upper layer, we perform prognostics and risk assessment in the framework's lower layer. To assess mission feasibility, we conducted 500 Monte Carlo simulations. We set a State of Charge (SoC) threshold (:math:`SoC_{\text{th}}`) of 30% for determining mission success or failure. SoC predictions for six flight missions are shown in Figure SoC_predction. For mission decision-making, we used Equation success_proba with the SoC Probability Density Function (PDF), :math:`SoC_{\text{th}}`, and a probability threshold (:math:`P_{\text{threshold}}`) of 95%. As per Algorithm ste-up, a mission is executed if the probability of arriving with an SoC of 30% is above 95%. The final decisions, are in the table below. 

.. list-table:: Mission Success Probabilities and Decisions
   :align: center
   :header-rows: 1
   :widths: auto

   * - **Mission ID**
     - **Success Probability**
     - **Decision**
   * - 1
     - 0.97
     - Cleared for Flight
   * - 2
     - 0.00
     - Hold
   * - 3
     - 1.00
     - Cleared for Flight
   * - 4
     - 0.10
     - Hold
   * - 5
     - 1.00
     - Cleared for Flight
   * - 6
     - 1.00
     - Cleared for Flight

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



