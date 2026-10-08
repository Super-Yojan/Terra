#!/usr/bin/env ruby
require 'xcodeproj'
root = File.expand_path('..', __dir__)
project_path = File.join(root, 'mobile/ios/TerraPhone.xcodeproj')
project = Xcodeproj::Project.new(project_path)
app = project.new_target(:application, 'TerraPhone', :ios, '17.0')
sources = project.main_group.new_group('TerraPhone', 'TerraPhone')
%w[TerraPhoneApp.swift ContentView.swift TerraDashboard.swift RoverModelView.swift TerraAutoConnectionPolicy.swift DriveControlPanel.swift DriveJoystickCommand.swift PhoneController.swift BluetoothSession.swift BluetoothLink.swift ActuatorConfiguration.swift ActuatorLayoutView.swift].each do |name|
  app.source_build_phase.add_file_reference(sources.new_file(name))
end
sources.new_file('Info.plist')
app.resources_build_phase.add_file_reference(sources.new_file('Assets.xcassets'))
models = sources.new_file('Models')
models.last_known_file_type = 'folder'
app.resources_build_phase.add_file_reference(models)
generated = project.main_group.new_group('Generated', 'Generated')
app.source_build_phase.add_file_reference(generated.new_file('TerraCore.swift'))
framework = generated.new_file('TerraCore.xcframework')
framework.last_known_file_type = 'wrapper.xcframework'
app.frameworks_build_phase.add_file_reference(framework)
%w[ARKit CoreMotion SwiftUI CoreBluetooth].each { |name| app.add_system_framework(name) }
app.build_configurations.each do |config|
  config.build_settings['PRODUCT_BUNDLE_IDENTIFIER'] = 'org.terra.robotics.phone'
  config.build_settings['INFOPLIST_FILE'] = 'TerraPhone/Info.plist'
  config.build_settings['SWIFT_VERSION'] = '5.0'
  config.build_settings['TARGETED_DEVICE_FAMILY'] = '1'
  config.build_settings['CODE_SIGN_STYLE'] = 'Automatic'
  config.build_settings['GENERATE_INFOPLIST_FILE'] = 'NO'
  config.build_settings['ENABLE_USER_SCRIPT_SANDBOXING'] = 'NO'
  config.build_settings['MARKETING_VERSION'] = '0.1.0'
  config.build_settings['CURRENT_PROJECT_VERSION'] = '1'
end
project.save
scheme = Xcodeproj::XCScheme.new
scheme.add_build_target(app)
scheme.set_launch_target(app)
scheme.save_as(project.path, 'TerraPhone', true)
