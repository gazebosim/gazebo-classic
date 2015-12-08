namespace gazebo
{
  namespace physics
  {
    class GearboxJointProtected
    {
      /// \brief Gearbox gearRatio
      public: double gearRatio;

      /// \brief reference link/body for computing joint angles
      public: std::string referenceBody;
    };
  }
}
