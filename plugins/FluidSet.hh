/* Desc: Fluid Set class
 * Author: Andrei Haidu
 * Date: 11 Jul. 2013
 */

#ifndef FLUID_SET_HH
#define FLUID_SET_HH

/// \brief namespace for fluids
namespace fluidix
{
    /// \class FluidSet FluidSet.hh
    /// \brief FluidSet class
    class FluidSet
    {
    	//TODO create constructors so that it initializes min values, and computes the rest
        /// \brief Constructor
        public: FluidSet();

        /// \brief Destructor
        public: virtual ~FluidSet();

        /// \brief Load
        public: virtual void Load();

        /// \brief Finalize
        public: virtual void Fini();

        /// \brief Initialize
        public: virtual void Init();

        /// \brief Reset
        public: virtual void Reset();

        /// \brief Update
        public: void Update();

        /// \brief Set particle set ID
        public: void SetParticleSetId(int _id);

        /// \brief Get particle set ID
        public: int GetParticleSetId();

        /// \brief Set the total number of particles in the fluid
        public: void SetParticleNr(int _nr);

        /// \brief Get the total number of particles in the fluid
        public: int GetParticleNr();

        /// \brief Set fluid volume (m3)
        public: void SetVolume(float _volume);

        /// \brief Get fluid volume (m3)
        public: float GetVolume();

        /// \brief Set particle size (m) as a side from a cube
        public: void SetParticleSize(float _size);

        /// \brief Get particle size (m) as a side from a cube
        public: float GetParticleSize();

        /// \brief Get particle volume (m3) as a side from a cube
        public: float GetParticleVolume();

        /// \brief Set fluid density (kg/m3)
        public: void SetDensity(float _density);

        /// \brief Get particle density (kg/m3)
        public: float GetDensity();

        /// \brief Set particle mass (kg)
        public: void SetParticleMass(float _mass);

        /// \brief Get particle mass (kg)
        public: float GetParticleMass();

        /// \brief Set Fluid mass (kg)
        public: void SetFluidMass(float _mass);

        /// \brief Get Fluid mass (kg)
        public: float GetFluidMass();

        /// \brief Set pressure stiffness constant (k)
        public: void SetStiffness(float _stiffness);

        /// \brief Get pressure stiffness constant (k)
        public: float GetStiffness();

        /// \brief Set fluid viscosity (Pa.s)
        public: void SetViscosity(float _viscosity);

        /// \brief Get fluid viscosity (Pa.s)
        public: float GetViscosity();

        /// \brief Set fluid surface tension (N/m)
        public: void SetSurfaceTension(float _tension);

        /// \brief Get fluid surface tension (N/m)
        public: float GetSurfaceTension();

        /// \brief Set fluid tension threshold
        public: void SetSurfTensThreshold(float _surfTensThreshold);

        /// \brief Get surface tension threshold
        public: float GetSurfTensThreshold();//TODO not sure if needed

        /// \brief Set gas buoyancy  (m/s2)
        public: void SetBuoyancy(float _buoyancy);

        /// \brief Get gas buoyancy  (m/s2)
        public: float GetBuoyancy();

        /// \brief Set smoothing lengt, h (m)
        public: void SetSmoothingLength(float _length);

        /// \brief Get smoothing lengt, h (m)
        public: float GetSmoothingLength();

        /// \brief Set avg. number of neighbors (for smoothing length)
        public: void SetSmoothingNeighborsNr(int _nr);

        /// \brief Get avg. number of neighbors (for smoothing length)
        public: int GetSmoothingNeighboursNr();


        /// \brief Unique ID of the particle Set
        protected: int particleSetId;

        /// \brief Total number of particles in the fluid
        protected: int particleNr;

        /// \brief Volume of the fluid
        protected: float volume;

        /// \brief Particle size (m) as a side from a cube
        protected: float particleSize;

        /// \brief Fluid density (kg/m3)
        protected: float density;

        /// \brief particle mass
        protected: float particleMass;

        /// \brief Fluid total mass (kg)
        protected: float fluidMass;

        /// \brief Fluid pressure stiffness constant (k)
        protected: float stiffness;

        /// \brief Fluid viscosity (Pa.s)
        protected: float viscosity;

        /// \brief Fluid surface tension (N/m)
        protected: float surfaceTension;

        /// \brief Fluid surface tension threshold
        protected: float surfTensThreshold;

        /// \brief Gas buoyancy  (m/s2)
        protected: float buoyancy;

        /// \briefSnoothing radius, h (m)
        protected: float smoothingLength;

        /// \brief Avg. number of neighbors (for smoothing length)
        protected: int smoothingNeighboursNr;
    };
}
#endif

