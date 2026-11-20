namespace chicken_coop
{
  class SensorListener
  {
  public:
    virtual ~SensorListener() = default;
    virtual void onTemperature(float t) {}
    virtual void onHumidity(float h) {}
    virtual void onPressure(float p) {}
    virtual void onAmoniaSensor(float a) {}
  };
} // namespace chicken_coop