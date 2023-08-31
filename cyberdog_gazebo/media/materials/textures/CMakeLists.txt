set (files
  beigeWall.jpg
  ceiling_tiled.jpg
  clouds.jpg
  dirt_diffusespecular.png
  dirt_normal.png
  flat_normal.png
  fungus_diffusespecular.png
  fungus_normal.png
  granite2.jpg
  granite.jpg
  grass_diffusespecular.png
  grass.jpg
  grass_normal.png
  hardwood_floor.jpg
  motorway.jpg
  paintedWall.jpg
  pioneerBody.jpg
  primary.jpg
  projection_filter.png
  random.png
  residential.jpg
  secondary.jpg
  sidewalk.jpg
  steps.jpeg
  stereo_projection_pattern_high_res.png
  stereo_projection_pattern_high_res_red.png
  terrain.png
  trunk.jpg
  white.bmp
  willowMap.png
  wood.jpg
  WoodPallet.png
)


INSTALL(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/media/materials/textures/)
