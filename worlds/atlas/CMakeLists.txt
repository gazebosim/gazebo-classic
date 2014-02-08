set (files
atlas_no_head.sdf   l_hand.stl   l_uarm.stl   r_foot.stl   r_talus.stl
atlas_no_head.urdf  l_larm.dae   l_uglut.dae  r_hand.dae   r_uarm.dae
atlas.urdf          l_larm.stl   l_uglut.stl  r_hand.stl   r_uarm.stl
ground.urdf         l_lglut.dae  l_uleg.dae   r_larm.dae   r_uglut.dae
head.dae            l_lglut.stl  l_uleg.stl   r_larm.stl   r_uglut.stl
head_old.dae        l_lleg.dae   mtorso.dae   r_lglut.dae  r_uleg.dae
head.stl            l_lleg.stl   mtorso.stl   r_lglut.stl  r_uleg.stl
l_clav.dae          l_scap.dae   pelvis.dae   r_lleg.dae   utorso.dae
l_clav.stl          l_scap.stl   pelvis.stl   r_lleg.stl   utorso_pack.dae
l_farm.dae          l_talus.dae  r_clav.dae   r_lug.dae    utorso.stl
l_farm.stl          l_talus.stl  r_clav.stl   r_lug.stl
l_foot.dae          ltorso.dae   r_farm.dae   r_scap.dae
l_foot.stl          ltorso.stl   r_farm.stl   r_scap.stl
l_hand.dae          l_uarm.dae   r_foot.dae   r_talus.dae
)

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/worlds/atlas)
