// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.utils;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

@Retention(RetentionPolicy.RUNTIME)
@Target({ElementType.FIELD, ElementType.METHOD})
public @interface AutoLogLevel {

  public enum Level {
    DebugSim,
    DebugReal,
    Sim,
    Sysid,
    Real;

    public boolean isAtLeast(Level other) {
      return this.compareTo(other) >= 0;
    }
  }

  public String key() default "";

  public Level level() default Level.Real;
}
