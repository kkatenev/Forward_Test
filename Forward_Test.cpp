#include <iostream>
#include <vector>
#include <cmath>
#include <chrono>
#include <thread>

class EngineSimulator {
private:
    double I;
    std::vector<double> M;
    std::vector<double> V;
    double Tperegrev;
    double HM;
    double HV;
    double C;
    double Tambient;
    double Tengine;
    double maxPower;
    double maxPowerSpeed;
    double time;

public:
    EngineSimulator(double inertia, const std::vector<double>& torque, const std::vector<double>& velocity,
        double overheatTemp, double hm, double hv, double c, double ambientTemp)
        : I(inertia), M(torque), V(velocity), Tperegrev(overheatTemp), HM(hm), HV(hv), C(c), Tambient(ambientTemp),
        Tengine(ambientTemp), maxPower(0.0), maxPowerSpeed(0.0), time(0.0) {}

    void simulateHeating() {
        for (size_t i = 1; i < M.size(); ++i) {
            double dt = V[i] - V[i - 1];
            double vh = M[i - 1] * HM + std::pow(V[i - 1], 2) * HV;
            double vc = C * (Tambient - Tengine);

            double dT = (vh - vc) * dt;
            Tengine += dT;
            time += dt;

            std::cout << "Time: " << time << " seconds | Engine Temp: " << Tengine << " C" << std::endl;

            if (Tengine >= Tperegrev) {
                std::cout << "Engine overheat detected! Time: " << time << " seconds" << std::endl;
                break;
            }
        }
    }

    void simulateMaxPower() {
        double maxPower = 0.0;
        double maxPowerSpeed = 0.0;

        for (size_t i = 1; i < M.size(); ++i) {
            double power = M[i - 1] * V[i - 1] / 1000.0;

            if (power > maxPower) {
                maxPower = power;
                maxPowerSpeed = V[i - 1];
            }

            std::cout << "Power: " << power << " | Rotational speed: " << V[i - 1] << std::endl;

            double nextPower = M[i] * V[i] / 1000.0;

            if (power >= maxPower && nextPower <= power) {
                std::cout << "Max power reached: " << maxPower << " kW at " << maxPowerSpeed << " rad/s" << std::endl;
                break;
            }
        }
    }
};

int main() {
    double I = 10.0;
    std::vector<double> M = { 20, 75, 100, 105, 75, 0 };
    std::vector<double> V = { 0, 75, 150, 200, 250, 300 };
    double Tperegrev = 110;
    double HM = 0.01;
    double HV = 0.0001;
    double C = 0.1;

    std::cout << "Enter test model(1/2/3):\n1. Heating Simulation \n2. Max Power Simulation\n3. Both" << std::endl;
    int num = 0;
    while (num != 1 && num != 2 && num != 3)
        std::cin >> num;


    double ambientTemp;
    std::cout << "Enter ambient temperature (C): ";
    std::cin >> ambientTemp;

    EngineSimulator simulator(I, M, V, Tperegrev, HM, HV, C, ambientTemp);


    switch (num)
    {
    case(1): {
        // Simulate heating
        std::cout << std::endl << "Heating Simulation:" << std::endl;
        simulator.simulateHeating();
        break;
    }
    case(2): {
        // Simulate max power
        std::cout << std::endl << "Max Power Simulation:" << std::endl;
        simulator.simulateMaxPower();
        break;
    }
    case(3): {
        // Simulate heating
        std::cout << std::endl << "Heating Simulation:" << std::endl;
        simulator.simulateHeating();

        // Reset engine temperature and time
        simulator = EngineSimulator(I, M, V, Tperegrev, HM, HV, C, ambientTemp);

        // Simulate max power
        std::cout << std::endl << "Max Power Simulation:" << std::endl;
        simulator.simulateMaxPower();
        break;
    }
    }

    return 0;
}
