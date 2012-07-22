#ifndef _MERGE_CP_HH_
#define _MERGE_CP_HH_

#include <OgreCompositorInstance.h>
#include <OgreCustomCompositionPass.h>


namespace gazebo
{
  namespace rendering
  {
    /// The render operation that will be called each frame in the custom
    //composition pass. This is the class that will send the actual render
    //calls of the spheres (point lights), cones (spotlights) and quads
    //(directional lights) after the GBuffer has been constructed
    class MergeRenderOperation :
      public Ogre::CompositorInstance::RenderSystemOperation
    {
      public: MergeRenderOperation(Ogre::CompositorInstance *_instance,
                                   const Ogre::CompositionPass *_pass);
    	
    	/// @copydoc CompositorInstance::RenderSystemOperation::execute
      public: virtual void execute(Ogre::SceneManager *_sm,
                                   Ogre::RenderSystem *_rs);
    
      public: virtual ~MergeRenderOperation();
    };
    
    /// The custom composition pass that is used for rendering the light
    /// geometry. This class needs to be registered with the CompositorManager
    class MergeCompositionPass : public Ogre::CustomCompositionPass
    {
      /// @copydoc CustomCompositionPass::createOperation
      public: virtual Ogre::CompositorInstance::RenderSystemOperation *
              CreateOperation(Ogre::CompositorInstance *_instance,
                              const Ogre::CompositionPass *_pass)
      {
        return OGRE_NEW MergeRenderOperation(instance, pass);
      }

      protected: virtual ~MergeCompositionPass() {}
    };
  }
}
#endif
