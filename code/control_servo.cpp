#include "control_servo.h"

double weighted_average(
    double v0, double v1, double v2,
    double w0, double w1, double w2) noexcept
{
    return v0 * w0 + v1 * w1 + v2 * w2;
}

bool readPIDParams(const std::string &filename, PIDParams &params)
{
    std::ifstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Error: Cannot open file " << filename << std::endl;
        return false;
    }

    std::string line;
    while (std::getline(file, line))
    {
        const size_t colon_pos = line.find(':');
        if (colon_pos == std::string::npos)
            continue;

        const std::string key = line.substr(0, colon_pos);
        const std::string val_str = line.substr(colon_pos + 1);

        try
        {
            const double value = std::stod(val_str);

            if (key == "p")
                params.p = value;
            else if (key == "i")
                params.i = value;
            else if (key == "d")
                params.d = value;
        }
        catch (const std::invalid_argument &e)
        {
            std::cerr << "Invalid number format: " << line << " (" << e.what() << ")\n";
            return false;
        }
        catch (const std::out_of_range &e)
        {
            std::cerr << "Number out of range: " << line << " (" << e.what() << ")\n";
            return false;
        }
    }
    return true;
}
