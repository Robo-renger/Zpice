#!/usr/bin/env python3
import csv
import os
import yaml
import pandas as pd
import json

class DataReader:
    def __init__(self, file_path):
        self.__file_path = file_path
        self.__year_data_dict = {}
        self.__data = []
        self.__expected_regions = 5  # Set the expected number of regions

        # Get file extension
        _, file_extension = os.path.splitext(file_path)
        self.__file_extension = file_extension.lower()
        
        self.load_data()
        self.load_year_data_dict()

    def load_data(self):
        """Dynamically load data based on file extension"""
        if self.__file_extension in ['.csv']:
            self.__load_csv()
        elif self.__file_extension in ['.xlsx', '.xls']:
            self.__load_excel()
        elif self.__file_extension in ['.tsv']:
            self.__load_tsv()
        elif self.__file_extension in ['.yaml', '.yml']:
            self.__load_yaml()
        elif self.__file_extension in ['.txt', '.log']:
            self.__load_text()
        elif self.__file_extension in ['.json']:
            self.__load_json()
        else:
            raise ValueError(f"Unsupported file format: {self.__file_extension}. Supported formats: .csv, .xlsx, .xls, .tsv, .yaml, .yml, .txt, .log, .json")

    def __load_csv(self):
        """Load data from CSV file"""
        with open(self.__file_path, mode='r', newline='', encoding='utf-8') as file:
            reader = csv.reader(file, delimiter=',')
            self.__data = list(reader)

    def __load_excel(self):
        """Load data from Excel file"""
        df = pd.read_excel(self.__file_path)
        
        # Convert the dataframe to a list of lists
        header = df.columns.tolist()
        rows = df.values.tolist()
        self.__data = [header] + rows

    def __load_tsv(self):
        """Load data from TSV file"""
        with open(self.__file_path, mode='r', newline='', encoding='utf-8') as file:
            reader = csv.reader(file, delimiter='\t')
            self.__data = list(reader)

    def __load_yaml(self):
        """Load data from YAML file"""
        with open(self.__file_path, 'r', encoding='utf-8') as file:
            yaml_data = yaml.safe_load(file)
            self.__data = self.__convert_to_table_format(yaml_data)

    def __load_json(self):
        """Load data from JSON file"""
        with open(self.__file_path, 'r', encoding='utf-8') as file:
            json_data = json.load(file)
            self.__data = self.__convert_to_table_format(json_data)

    def __load_text(self):
        """Load data from text file - tries to detect format"""
        with open(self.__file_path, 'r', encoding='utf-8') as file:
            content = file.readlines()
            
            # Try to determine the delimiter
            if content:
                first_line = content[0].strip()
                
                # Check for CSV format
                if ',' in first_line:
                    delimiter = ','
                # Check for TSV format
                elif '\t' in first_line:
                    delimiter = '\t'
                # Check for space-delimited format
                elif ' ' in first_line and len(first_line.split()) > 1:
                    delimiter = None  # Split on any whitespace
                else:
                    # Default to comma if format can't be determined
                    delimiter = ','
                
                # Parse the file with the detected delimiter
                if delimiter is None:
                    self.__data = [line.strip().split() for line in content if line.strip()]
                else:
                    self.__data = [line.strip().split(delimiter) for line in content if line.strip()]
            else:
                self.__data = []

    def __convert_to_table_format(self, structured_data):
        """Convert structured data from JSON/YAML to tabular format"""
        table_data = []
        
        # Case 1: Dictionary with years as keys
        if isinstance(structured_data, dict) and any(str(k).isdigit() for k in structured_data.keys()):
            # Create header
            header = ['year']
            for i in range(self.__expected_regions):
                header.append(f'region{i+1}')
            table_data.append(header)
            
            # Add data rows
            for year, year_data in structured_data.items():
                if not str(year).isdigit():
                    continue
                    
                row = [year]
                if isinstance(year_data, dict):
                    # Handle dictionary format (region keys)
                    for i in range(self.__expected_regions):
                        region_key = f'region{i+1}'
                        if region_key in year_data:
                            row.append(year_data[region_key])
                        else:
                            row.append('')
                elif isinstance(year_data, list):
                    # Handle list format
                    row.extend(year_data[:self.__expected_regions])
                    # Pad if needed
                    row.extend([''] * (self.__expected_regions - len(year_data)))
                else:
                    # Unknown format
                    row.extend([''] * self.__expected_regions)
                
                table_data.append(row)
            
            return table_data
        
        # Case 2: List of dictionaries with year field
        elif isinstance(structured_data, list) and all(isinstance(item, dict) for item in structured_data):
            # Look for year field
            year_keys = ['year', 'Year', 'YEAR', 'id', 'ID', 'Id']
            
            # Create header
            header = ['year']
            for i in range(self.__expected_regions):
                header.append(f'region{i+1}')
            table_data.append(header)
            
            # Add data rows
            for item in structured_data:
                year = None
                for key in year_keys:
                    if key in item:
                        year = item[key]
                        break
                        
                if year is None or not str(year).isdigit():
                    continue
                    
                row = [year]
                for i in range(self.__expected_regions):
                    region_key = f'region{i+1}'
                    if region_key in item:
                        row.append(item[region_key])
                    else:
                        row.append('')
                        
                table_data.append(row)
                
            return table_data
            
        # Case 3: List of lists (already tabular)
        elif isinstance(structured_data, list) and all(isinstance(item, list) for item in structured_data):
            # Check if first row looks like a header
            if structured_data and len(structured_data) > 1:
                # Use as is, but ensure it has expected number of columns
                header = structured_data[0]
                if len(header) < 1 + self.__expected_regions:
                    # Extend header if needed
                    current_len = len(header)
                    if current_len == 0:
                        header = ['year']
                    else:
                        header = header[:1]  # Keep just the year column
                        
                    for i in range(self.__expected_regions):
                        header.append(f'region{i+1}')
                
                table_data.append(header)
                
                # Process data rows
                for row in structured_data[1:]:
                    if not row:
                        continue
                        
                    # Ensure first column is year
                    if not str(row[0]).isdigit():
                        continue
                        
                    processed_row = [row[0]]
                    if len(row) > 1:
                        processed_row.extend(row[1:1+self.__expected_regions])
                    
                    # Pad if needed
                    processed_row.extend([''] * (1 + self.__expected_regions - len(processed_row)))
                    
                    table_data.append(processed_row)
                
                return table_data
        
        # Fallback: create a simple table with header
        header = ['year']
        for i in range(self.__expected_regions):
            header.append(f'region{i+1}')
        table_data.append(header)
        
        return table_data

    def get_row(self, year):
        """Get data for a specific year"""
        try:
            return self.__year_data_dict[str(year)]
        except KeyError:
            return "No Such year exists"
    
    def load_year_data_dict(self):
        """Process the loaded data and organize it by year"""
        if not self.__data:
            raise ValueError("No data was loaded from the file or the file is empty")
        
        # Skip header row (index 0)
        for row in self.__data[1:]:
            # Skip empty rows
            if not row or not row[0]:
                continue
                
            # Check if the first column is a year
            if str(row[0]).isdigit():
                year = str(row[0])
                
                # Ensure we have enough columns
                data_values = row[1:]
                if len(data_values) < self.__expected_regions:
                    data_values.extend([''] * (self.__expected_regions - len(data_values)))
                
                # Initialize year data
                self.__year_data_dict[year] = [False] * self.__expected_regions
                
                # Process each region value
                for i in range(self.__expected_regions):
                    if i < len(data_values):
                        value = data_values[i]
                        
                        # Skip empty values
                        if value == '':
                            continue
                            
                        # Normalize the value to a string
                        norm_value = str(value).lower().strip()
                        
                        # Convert to boolean based on value
                        if norm_value =='y':
                            self.__year_data_dict[year][i] = True
                        elif norm_value =='n':
                            self.__year_data_dict[year][i] = False
                        else:
                            raise ValueError(f"Invalid data at column of region {i+1} for year {year}. Data must be y/n, yes/no, true/false, or 1/0.")

    def get_years(self):
        """Return all available years"""
        return self.__year_data_dict.keys()