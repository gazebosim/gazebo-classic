namespace gazebo
{
  namespace physics
  {
    class GearboxJointPrivate
    {
      /// \brief Gearbox gearRatio
      public: double gearRatio = 1.0;

      /// \brief reference link/body for computing joint angles
      public: std::string referenceBody;
    };
  }
}
