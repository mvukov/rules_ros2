"""Provides rules to build targets with qt."""

load("@com_github_mvukov_rules_ros2//ros2:cc_defs.bzl", "ros2_cpp_library")


def qt_ui_library(name, ui, deps):
  """Compiles a QT UI file and makes a library for it.
  Args:
    name: A name for the rule.
    src: The ui file to compile.
    deps: cc_library dependencies for the library.
  """
  native.genrule(
      name = "%s_uic" % name,
      srcs = [ui],
      outs = ["ui_%s.h" % ui.split('.')[0]],
      cmd = "uic $(locations %s) -o $@" % ui,
  )

  native.cc_library(
      name = name,
      hdrs = [":%s_uic" % name],
      deps = deps,
  )

def qt_cc_library(name, src, hdr, normal_hdrs=[], deps=None, ui=None,
                  ui_deps=None, **kwargs):
  """Compiles a QT library and generates the MOC for it.
  If a UI file is provided, then it is also compiled with UIC.
  Args:
    name: A name for the rule.
    src: The cpp file to compile.
    hdr: The single header file that the MOC compiles to src.
    normal_hdrs: Headers which are not sources for generated code.
    deps: cc_library dependencies for the library.
    ui: If provided, a UI file to compile with UIC.
    ui_deps: Dependencies for the UI file.
    kwargs: Any additional arguments are passed to the cc_library rule.
  """
  native.genrule(
      name = "%s_moc" % name,
      srcs = [hdr],
      outs = ["moc_%s.cpp" % name],
      cmd =  "moc $(location %s) -o $@ -f'%s'" \
        % (hdr, '%s/%s' % (native.package_name(), hdr)),
  )
  srcs = [src, ":%s_moc" % name]

  if ui:
    qt_ui_library("%s_ui" % name, ui, deps=ui_deps)
    deps.append("%s_ui" % name)

  hdrs = [hdr] + normal_hdrs

  native.cc_library(
      name = name,
      srcs = srcs,
      hdrs = hdrs,
      deps = deps,
      **kwargs
  )

def ros_qt_cc_library(name, src, hdr, normal_hdrs=[], deps=None, ui=None,
                  ui_deps=None, **kwargs):
  """Compiles a QT library in combination with ROS and generates the MOC for it.
  If a UI file is provided, then it is also compiled with UIC.
  Args:
    name: A name for the rule.
    src: The cpp file to compile.
    hdr: The single header file that the MOC compiles to src.
    normal_hdrs: Headers which are not sources for generated code.
    deps: cc_library dependencies for the library.
    ui: If provided, a UI file to compile with UIC.
    ui_deps: Dependencies for the UI file.
    kwargs: Any additional arguments are passed to the cc_library rule.
  """
  native.genrule(
      name = "%s_moc" % name,
      srcs = [hdr],
      outs = ["moc_%s.cpp" % name],
      cmd =  "moc $(location %s) -o $@ -f'%s'" \
        % (hdr, '%s/%s' % (native.package_name(), hdr)),
  )
  srcs = [src, ":%s_moc" % name]

  if ui:
    qt_ui_library("%s_ui" % name, ui, deps=ui_deps)
    deps.append("%s_ui" % name)

  hdrs = [hdr] + normal_hdrs

  ros2_cpp_library(
      name = name,
      srcs = srcs,
      hdrs = hdrs,
      deps = deps,
      **kwargs
  )