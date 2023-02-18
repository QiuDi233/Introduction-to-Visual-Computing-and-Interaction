#include "Labs/5-Visualization/tasks.h"

#include <numbers>

using VCX::Labs::Common::ImageRGB;
namespace VCX::Labs::Visualization {

    struct CoordinateStates {
        // your code here
        std::vector<Car>data;
    };

    bool PaintParallelCoordinates(Common::ImageRGB & input, InteractProxy const & proxy, std::vector<Car> const & data, bool force) {
        // your code here
        // for example: 
        //   static CoordinateStates states(data);
        //   SetBackGround(input, glm::vec4(1));
        //   ...
        static CoordinateStates states(data);
        SetBackGround(input, glm::vec4(1));//»­°×É«±³¾°

        float x[7];
        for (int i = 0; i < 6; ++i) {
            x[i] = i*0.9 / 6.0 + 0.05;
        }
        x[6] = 0.95;
        for (int i = 0; i < 7; ++i) {
            DrawLine(input, glm::vec4(0.5, 0.5, 0.5, 1), glm::vec2(x[i], 0.1), glm::vec2(x[i], 0.9), 10);
        }
        std::string type[7] = { "cylinders", "displacement", "weight", "horsepower", "acceleration(0-60mph)", "mileage", "year" };
        std::string num1[7] = { "9", "494", "5493", "249", "27", "51", "84" };
        std::string num2[7] = { "2", "29", "1260", "27", "6", "5", "68" };
        for (int i = 0; i < 7; ++i) {
            PrintText(input, glm::vec4(0.0, 0.0, 0.0, 1), glm::vec2(x[i],0.05), 0.02, type[i]);
            PrintText(input, glm::vec4(0.0, 0.0, 0.0, 1), glm::vec2(x[i]-0.005, 0.08), 0.02, num1[i]);
            PrintText(input, glm::vec4(0.0, 0.0, 0.0, 1), glm::vec2(x[i]-0.005, 0.92), 0.02, num2[i]);
        }
        int n = data.size();
        for (int i = 0; i < data.size(); ++i) {
            DrawLine(input, glm::vec4(1.0 * i / n, 0.4, 0.5, 1), glm::vec2(x[0], 0.1 + 0.8 * (data[i].cylinders - 2) / (9 - 2)), glm::vec2(x[1], 0.1 + 0.8 * (data[i].displacement - 29) / (494 - 29)), 1);
            DrawLine(input, glm::vec4(1.0 * i / n, 0.4, 0.5, 1), glm::vec2(x[1], 0.1 + 0.8 * (data[i].displacement - 29) / (494 - 29)), glm::vec2(x[2], 0.1 + 0.8 * (data[i].weight - 1260) / (5493 - 1260)), 1);
            DrawLine(input, glm::vec4(1.0 * i / n, 0.4, 0.5, 1), glm::vec2(x[2], 0.1 + 0.8 * (data[i].weight - 1260) / (5493 - 1260)), glm::vec2(x[3], 0.1 + 0.8 * (data[i].horsepower - 27) / (249 - 27)), 1);
            DrawLine(input, glm::vec4(1.0 * i / n, 0.4, 0.5, 1), glm::vec2(x[3], 0.1 + 0.8 * 0.1 + 0.8 * (data[i].horsepower - 27) / (249 - 27)), glm::vec2(x[4], 0.1 + 0.8 * (data[i].acceleration - 6) / (27 - 6)), 1);
            DrawLine(input, glm::vec4(1.0 * i / n, 0.4, 0.5, 1), glm::vec2(x[4], 0.1 + 0.8 * (data[i].acceleration - 6) / (27 - 6)), glm::vec2(x[5], 0.1 + 0.8 * (data[i].mileage - 5) / (51 - 5)), 1);
            DrawLine(input, glm::vec4(1.0 * i / n, 0.4, 0.5, 1), glm::vec2(x[5], 0.1 + 0.8 * (data[i].mileage - 5) / (51 - 5)), glm::vec2(x[6], 0.1 + 0.8 * (data[i].year - 68) / (84 - 68)), 1);
        }
        return true;
    }

    void LIC(ImageRGB & output, Common::ImageRGB const & noise, VectorField2D const & field, int const & step) {
        // your code here
    }
}; // namespace VCX::Labs::Visualization