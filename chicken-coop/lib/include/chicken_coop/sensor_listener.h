namespace chicken_coop
{
    class SensorListener
    {
    public:
        virtual ~SensorListener() = default;
        virtual void onTemperature(float t) {}
        virtual void onHumidity(float h) {}
        virtual void onPressure(float p) {}
    };
}