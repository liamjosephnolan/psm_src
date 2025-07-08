#include "config.h"
#include "traj_data.h"
#include <cstring> // For strtok_r
#include <cstdlib> // For strtod



// Function to parse the trajectory data from the CSV string provided in traj_data.h.
// This function reads the raw CSV data, processes each line to extract time, x, y, and z values,
// and then converts the absolute timestamps into relative times, with the first point marking the start (time = 0).
void parse_traj_file() {
    // Create a mutable copy of the const char array, as strtok_r modifies the string.
    char* csv_copy = strdup(traj_csv); 
    if (csv_copy == NULL) {
        // Handle memory allocation error if necessary
        return;
    }

    char *line_saveptr; // Pointer for strtok_r to save its internal state for lines
    char *field_saveptr; // Pointer for strtok_r to save its internal state for fields

    // Get the first line (header) and discard it
    char* line = strtok_r(csv_copy, "\n", &line_saveptr); 
    line = strtok_r(NULL, "\n", &line_saveptr); // Get the first data line

    double first_time = -1.0; // Initialize to a sentinel value to identify the first timestamp.

    while (line != NULL) {
        TrajPoint point; // A struct to hold the data for a single trajectory point.

        // Parse time
        char* token = strtok_r(line, ",", &field_saveptr);
        if (token != NULL) {
            double current_time = strtod(token, NULL);
            if (first_time < 0) {
                first_time = current_time;
            }
            point.time = current_time - first_time;
        }

        // Parse x
        token = strtok_r(NULL, ",", &field_saveptr);
        if (token != NULL) {
            point.x = strtod(token, NULL);
        }

        // Parse y
        token = strtok_r(NULL, ",", &field_saveptr);
        if (token != NULL) {
            point.y = strtod(token, NULL);
        }

        // Parse z
        token = strtok_r(NULL, ",", &field_saveptr);
        if (token != NULL) {
            point.z = strtod(token, NULL);
        }

        trajectory.push_back(point);
        line = strtok_r(NULL, "\n", &line_saveptr); // Get the next line
    }

    free(csv_copy); // Free the duplicated string memory
}