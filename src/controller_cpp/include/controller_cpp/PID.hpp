#include <string>

class PID {
  public:
    /**
     * @brief Default Constructor
     *
     */
    PID();

    /**
     * @brief Construct a new PID object
     *
     * @param p
     * @param i
     * @param d
     * @param feed_forward
     * @param start_i
     * @param name
     */
    PID(double kp, double ki, double kd, double kf, double start_i, std::string name = "");

    /**
     * @brief Set the PID constants
     *
     * @param p
     * @param i
     * @param d
     * @param feed_forward
     * @param start_i
     */
    void set_constants(double p, double i = 0, double d = 0, double f = 0, double start_i = 0);

    /**
     * @brief data struct for PID constants
     *
     */
    struct Constants {
        double kp;
        double ki;
        double kd;
        double kf;
        double start_i;
    } constants;

    /**
     * @brief Set the PID target
     *
     * @param input
     */
    void set_target(double target);

    /**
     * @brief compute PID
     *
     * @param input
     */
    double compute(double input, double dt);

    /**
     * @brief return target
     *
     */
    double return_target();

    /**
     * @brief return PID constants
     *
     * @return Constants
     */
    Constants return_constants();

    /**
     * @brief reset PID variables
     *
     */
    void reset_variables();

    /**
     * @brief set PID object name
     *
     */
    void set_name(std::string name);

    /**
     * \brief PID variables
     */
    double integral;
    double derivative;
    double output;
    double target;
    double error;
    double prev_error;

  private:
    void rest_timers();
    std::string name;
    bool is_name = false;
};