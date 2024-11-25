import os
from plots_for_paper import PlotsPaper 

class MissionRiskAssessment(PlotsPaper):
    def __init__(self, pickle_file_path, threshold):
        super().__init__(pickle_file_path)
        self.num_traces = 500 
        self.threshold = threshold
        self.mission_probabilities = {}

    def compute_mission_success_probability(self):
        SOC_params = self.get_SOC_forall_pickles()
        probability_success = []
        for i, profile_dict in enumerate(SOC_params):
            for profile_name, soc_profile in profile_dict.items():
                # Extract the last element of each profile as SOC
                SOC = [profile[-1] for profile in soc_profile]
                success_count = sum(soc > self.threshold for soc in SOC)
                success_probability = success_count / len(SOC)                
                probability_success.append(success_probability)
                    
        return probability_success

    def assess_missions(self):
        # TDOO: add more features here in the future 
        pass 

    def make_decision(self):
        probability_success = self.compute_mission_success_probability()
        print("Mission ID | Success Probability | Decision")
        print("--------------------------------------------")
        for mission_id, probability in enumerate(probability_success):
            decision = "Cleared for Flight" if probability >= self.threshold else "Hold"
            print(f"{mission_id:<11} | {probability:.2f}                 | {decision}")


if __name__ == "__main__":
    current_directory = os.path.dirname(os.path.abspath(__file__))  # Get the current directory of the script
    pickle_file_path = os.path.join(current_directory, "..", "BatteryPrognosticsResults/Pickles")
    threshold = 0.4  
    mission_assessment = MissionRiskAssessment(pickle_file_path, threshold)
    mission_assessment.make_decision()