import json

import numpy as np
import pandas as pd
from itertools import product


class Fuzzer:

    # ========================= 1. Initialization ===============================================================

    def __init__(self, json_path, csv_path, inputs=[], outputs=[]):
        """
        Initialize the ArrayStorage class with a dictionary of 2D arrays.
        The arrays are converted immediately upon initialization and results are generated.

        Args:
            data_dict (dict): A dictionary containing nested data structures to be converted to 2D arrays.
            csv_path (str): Path to the CSV file used for initializing the rule base.
            inputs (list, optional): A list of input endpoints. Defaults to an empty list.
            outputs (list, optional): A list of output endpoints. Defaults to an empty list.
        """
        self.rule_base = self.initialize_rule_base(csv_path, json_path)
        self.inputs = self.initialise_endpoints(inputs)
        self.outputs = self.initialise_endpoints(outputs)
        self.arrays = self.convert_to_2d_arrays(json_path)  # Convert data_dict to 2D arrays
        self.results = self.generate_results_array(inputs, outputs)  # Generate results based on arrays
        self.firing_strength = []

    def initialize_rule_base(self, csv_path, json_path):
        """
        Initializes the rule base by reading a CSV file and encoding its columns
        based on the provided dictionary.

        Args:
            csv_path (str): Path to the CSV file to be read.
            data_dict (dict): A dictionary mapping column names to encoding dictionaries.

        Returns:
            pd.DataFrame: A DataFrame with columns encoded as per the provided mappings.
        """

        with open(json_path, 'r') as file:
            data_dict = json.load(file)

        df = pd.read_csv(csv_path, header=0)

        for column, encoding in data_dict.items():
            df[column] = df[column].apply(lambda x: encoding[x])

        return df

    def initialise_endpoints(self, input_keys):
        """
        Initializes a dictionary with the input keys, setting each value to 0.0.

        Args:
            input_keys (list): A list of input keys to initialize.

        Returns:
            dict: A dictionary with each input key mapped to 0.0.
        """
        input = {}
        for input_key in input_keys:
            input[input_key] = 0.0

        return input

    # ========================= 2. Data Conversion and Results Generation =======================================

    def convert_to_2d_arrays(self, json_path):
        """
        Converts a nested dictionary to a dictionary of 2D arrays by extracting values from sub-dictionaries.

        Args:
            data_dict (dict): The original dictionary with nested structures.

        Returns:
            dict: A dictionary where each key is associated with a 2D array consisting of the values from the sub-dictionaries.
        """

        with open(json_path, 'r') as file:
            data_dict = json.load(file)

        result_2d_arrays = {}
        for main_key, sub_dict in data_dict.items():
            result_2d_arrays[main_key] = list(sub_dict.values())  # Convert nested values to a list (2D array)
        return result_2d_arrays

    def generate_results_array(self, inputs, outputs):
        """
        Generates a results dictionary with keys based on the input and output array keys,
        where each key corresponds to a list or default value for the results.

        Args:
            inputs (list): A list of input keys used to generate results.
            outputs (list): A list of output keys used to generate results.

        Returns:
            dict: A dictionary where each key corresponds to a list or value initialized for results.
        """
        results = {}

        for input_key in inputs:
            result_key = f"{input_key}_results"
            results[result_key] = [None] * 4

        for output_key in outputs:
            result_key = f"{output_key}_results"
            results[result_key] = 0.0

        return results

    def get_array(self, key):
        """
        Retrieve the 2D array associated with the specified key.

        Args:
            key (str): The key to retrieve the 2D array for.

        Returns:
            list: The 2D array of values associated with the key, or None if the key does not exist.
        """
        return self.arrays.get(key)

    def get_all_arrays(self):
        """
        Retrieve all 2D arrays stored in the class.

        Returns:
            dict: A dictionary of all 2D arrays.
        """
        return self.arrays

    def get_result(self, key):
        """
        Retrieve the 2D results associated with the specified key.

        Args:
            key (str): The key to retrieve the 2D array for.

        Returns:
            list: The 2D array of values associated with the key, or None if the key does not exist.
        """
        result_key = f"{key}_results"

        return self.results.get(result_key)

    def get_results_array(self):
        """
        Retrieve the generated results dictionary.

        Returns:
            dict: The results dictionary with dynamically formatted keys.
        """
        return self.results

    def get_rule_base(self):
        """
        Retrieves the rule base DataFrame.

        Returns:
            pd.DataFrame: The DataFrame containing the rule base.
        """
        return self.rule_base

    def get_resulting_arrays(self):
        """
        Retrieves the resulting arrays and fuzzy values for the inputs.

        The resulting arrays are the first two elements from the result for each input key,
        and the fuzz values are the last N elements, where N is the number of output keys.

        Returns:
            tuple: A tuple containing two lists:
                - resulting_arrays: The first two elements of the results for each input.
                - fuzz_values: The last N elements of the results for each input, where N is the number of output keys.
        """
        resulting_arrays = []
        fuzz_values = []

        for key, _ in self.inputs.items():
            resulting_arrays.append([item for item in self.get_result(key)[:2] if item is not None])
            result_values = self.get_result(key)[2:]
            filtered_values = [value for value in result_values if value is not None]
            fuzz_values.append(filtered_values)
            # fuzz_values.append([value for value in self.get_result(key)[-2:] if value is not None])

        return resulting_arrays, fuzz_values

    # ===================================== 3. Fuzzification ====================================================

    def fuzzification(self):
        """
        Applies fuzzification to each input, updating the results dictionary
        with the fuzzy values for each input key.
        """
        for key, value in self.inputs.items():
            self.results[f'{key}_results'] = self.fuzzify_input(key, value)

    def fuzzify_input(self, key, input_value):
        """
        Evaluate the input value using all provided arrays and determine
        the rising and falling edge values based on the input.

        Args:
            key (str): The input key for which fuzzification is performed.
            input_value (float): The value to be fuzzified.

        Returns:
            list: A list containing the falling and rising edges, or None if no match is found.
        """
        arrays = self.get_array(key)
        true_out = [None, None, 0, 0]

        for i in range(len(arrays) - 1):
            falling = arrays[i]
            rising = arrays[i + 1]

            rising_bool = rising[0] <= input_value <= rising[-1]
            falling_bool = falling[0] <= input_value <= falling[-1]

            if rising_bool and falling_bool:
                edge = self.calculate_edges(falling, rising, input_value)

                return edge

            if rising_bool:
                true_out = [None, rising, 1]

            if falling_bool:
                true_out = [falling, None, 1]

        return true_out

    def find_limits(self, values=[], input_value=0.0):
        """
        Finds the lower and higher limits based on the given values using boolean indexing.

        Args:
            values (list or np.array): The list or array of values to compare against the input value.
            input_value (float): The value to compare to find the lower and higher limits.

        Returns:


def clbk_laser(msg):
    global regions_, twstmsg_

    regions_ = {
        #LIDAR readings are anti-clockwise
        'front1':  find_nearest (msg.ranges[0:15]),
        'front2':  find_nearest (msg.ranges[345:360]),
        'frontR': find_nearest(msg.ran
            tuple: A tuple containing the lower and higher limits, or (-1, -1) if limits cannot be found.
        """
        values = np.array(values)

        lower_limit_candidates = values[values < input_value]
        higher_limit_candidates = values[values > input_value]

        lower_limit = lower_limit_candidates[-1] if lower_limit_candidates.size > 0 else None
        higher_limit = higher_limit_candidates[0] if higher_limit_candidates.size > 0 else None

        if lower_limit is None:
            return 0, higher_limit

        elif higher_limit is None:
            return lower_limit, 0

        return lower_limit, higher_limit

    def calculate_edges(self, falling, rising, input_value):
        """
        Calculates the membership function (MF) values for the rising and falling values,
        with protection against zero-division errors.

        Args:
            rising (list or np.array): The list or array of rising values to calculate the MF for.
            falling (list or np.array): The list or array of falling values to calculate the MF for.
            input_value (float): The input value used to calculate the MF.

        Returns:
            tuple: A tuple containing the calculated rise and fall MF values.
        """

        if rising[1] == input_value:
            return [None, rising, 1]
        elif falling[1] == input_value:
            return [falling, None, 1]

        r_min, r_max = self.find_limits(rising, input_value)
        f_min, f_max = self.find_limits(falling, input_value)

        fall_val = (input_value - f_min) / (f_max - f_min)
        rise_val = (r_max - input_value) / (r_max - r_min)

        # if f_max != f_min:
        #     fall_val = (input_value - f_min) / (f_max - f_min)
        # else:
        #     fall_val = 1 if input_value == f_min else 0

        # if r_max != r_min:
        #     rise_val = (r_max - input_value) / (r_max - r_min)
        # else:
        #     rise_val = 1 if input_value == r_min else 0

        return [falling, rising, fall_val, rise_val]

    # ===================================== 4. Defuzzification ==================================================

    def defuzzification(self):
        """
        Perform the defuzzification process by updating the outputs based on the rule base and firing strength.

        This method iterates through the rows in the rule base, calculates the centroid of the fuzzy set
        for each output, and updates the outputs by considering the firing strength of the rule.
        Finally, it normalizes the output values.
        """
        outputs = self.initialise_endpoints(list(self.outputs.keys()))
        for index, row in self.match_rule_base().iterrows():
            for key, _ in self.outputs.items():
                centroid = sum(row[key][1:-1]) / len(row[key][1:-1])
                outputs[key] += centroid * self.firing_strength[index]

        for key, _ in self.outputs.items():
            self.set_output(key, (outputs[key] / sum(self.firing_strength)))

    # ===================================== 5. Rule Base Matching ===============================================

    def get_combinations(self):
        """
        Generate all combinations of elements from the provided nested arrays.

        Returns:
            list of tuples: All possible combinations of elements.
        """
        arrays, fuzz_values = self.get_resulting_arrays()
        if not arrays or any(not arr for arr in arrays):
            return []

        self.set_firing_strength(fuzz_values)
        combinations = [list(comb) for comb in product(*arrays)]
        return combinations, len(combinations[0])

    def match_rule_base(self):
        """
        Find rows in the DataFrame that match any of the given combinations.

        Returns:
            pd.DataFrame: DataFrame containing the matching rows with header names.
        """
        results = []
        combinations, length = self.get_combinations()
        combinations_set = set(tuple(map(tuple, comb)) for comb in combinations)

        for idx, row in self.rule_base.iterrows():
            row_tuple = tuple(map(tuple, row.iloc[:length]))

            if row_tuple in combinations_set:
                results.append(row.tolist())

        matched_df = pd.DataFrame(results, columns=self.rule_base.columns)
        return matched_df.reset_index()

    def set_firing_strength(self, fuzz_values):
        """
        Calculate the firing strength based on the fuzzification values.

        Args:
            fuzz_values (list): List of fuzzified values for the inputs.
        """
        self.firing_strength = [min(value) for value in product(*fuzz_values)]

    def get_firing_strength(self):
        """
        Retrieve the current firing strength values.

        Returns:
            list: List of firing strength values.
        """
        return self.firing_strength

    # ===================================== 6. Input and Output Management ======================================

    def set_input(self, key, value):
        """
        Sets the value for a specific input key.

        Args:
            key (str): The key to set the value for.
            value (any): The value to associate with the input key.
        """
        self.inputs[key] = value

    def get_inputs(self):
        """
        Retrieves the dictionary of input keys and their associated values.

        Returns:
            dict: A dictionary of input keys with their associated values.
        """
        return self.inputs

    def set_output(self, key, value):
        """
        Sets the value for a specific output key and updates the results.

        Args:
            key (str): The output key to set the value for.
            value (any): The value to associate with the output key.
        """
        self.outputs[key] = value
        self.results[f'{key}_results'] = value

    def get_outputs(self):
        """
        Retrieves the dictionary of output keys and their associated values.

        Returns:
            dict: A dictionary of output keys with their associated values.
        """
        return self.outputs

    # ===================================== 7. Evaluation and Miscellaneous =====================================

    def __call__(self):
        """
        Perform fuzzification and defuzzification processes for the system evaluation.
        """
        self.fuzzification()
        self.defuzzification()


class FuzzyController:
    def __init__(self, fuzzy_models, outputs):
        """
        Initialize the FuzzyController with fuzzy models and outputs.

        Args:
            fuzzy_models (dict): Dictionary containing fuzzy models.
            outputs (list): List of output keys to calculate.

        Raises:
            ValueError: If the structure of fuzzy_models is invalid.
        """
        self.validate_fuzzy_models(fuzzy_models)
        self.fuzzy_models = fuzzy_models
        self.outputs = outputs
        print('done')

    @staticmethod
    def validate_fuzzy_models(fuzzy_models):
        """
        Validates the structure of the fuzzy_models dictionary.

        Args:
            fuzzy_models (dict): Dictionary of fuzzy models to validate.

        Raises:
            ValueError: If the structure of fuzzy_models is invalid.
        """
        for key, value in fuzzy_models.items():
            if not isinstance(value, dict):
                raise ValueError(f"Fuzzy model '{key}' must be a dictionary.")
            if 'model' not in value or 'ranges' not in value:
                raise ValueError(f"Fuzzy model '{key}' must contain 'model' and 'ranges' keys.")
            if not isinstance(value['ranges'], list) or len(value['ranges']) != 3:
                raise ValueError(f"Fuzzy model '{key}' must have 'ranges' as a list of three values.")
            if not all(isinstance(r, (int, float)) for r in value['ranges']):
                raise ValueError(f"Fuzzy model '{key}' ranges must contain numeric values.")

    @staticmethod
    def get_inputs(model_in_keys, inputs):
        model_inputs = {key: inputs[key] for key in model_in_keys if key in inputs}
        missing_keys = [key for key in model_in_keys if key not in inputs]
        if missing_keys:
            raise KeyError(f"Missing required input keys: {missing_keys}")
        return model_inputs

    def tri_mf_control(self, inputs):
        """
        Controls the turtlebot using triangular membership functions.

        Args:
            inputs (dict): Dictionary of sensor inputs.

        Returns:
            dict: Decisions for the specified output keys.
        """
        min_values = {}
        for key, obj in self.fuzzy_models.items():
            model_inputs = self.get_inputs(obj['model'].inputs, inputs)
            for input_key, input_value in model_inputs.items():
                obj['model'].set_input(input_key, input_value)
            obj['model']()
            min_values[key] = np.min(list(obj['model'].get_inputs().values()))

        membership = {}
        max_membership_key = None
        max_range_value = float('-inf')
        for key, obj in self.fuzzy_models.items():
            mid_range = obj['ranges'][1]
            if mid_range <= min_values[key] <= max(obj['ranges']):
                f_min, f_max = obj['model'].find_limits(obj['ranges'], min_values[key])
                membership[key] = (min_values[key] - f_min) / (f_max - f_min)
            elif obj['ranges'][0] <= min_values[key] < mid_range:
                membership[key] = 1
            else:
                membership[key] = 0
            if max(obj['ranges']) > max_range_value:
                max_range_value = max(obj['ranges'])
                max_membership_key = key

        if all(value == 0 for value in membership.values()) and max_membership_key is not None:
            membership[max_membership_key] = 1

        decisions = {output_key: 0 for output_key in self.outputs}
        for key, obj in self.fuzzy_models.items():
            model_output = obj['model'].outputs
            firing_strength = membership[key]
            for output_key in self.outputs:
                if output_key in obj['negate']:
                    decisions[output_key] += firing_strength * (-1 * model_output.get(output_key, 0))
                else:
                    decisions[output_key] += firing_strength * model_output.get(output_key, 0)

        firing_strengths = sum(membership.values())
        if firing_strengths > 0:
            for output_key in self.outputs:
                decisions[output_key] /= firing_strengths

        return decisions

