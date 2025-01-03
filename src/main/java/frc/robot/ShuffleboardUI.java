package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public enum ShuffleboardUI {
    Autonomous("Autonomous"),
    Overview("Overview"),
    Test("Test")
    ;

    private final String m_tabName;

    ShuffleboardUI(String tabName) {
        m_tabName = tabName;
    }

    public String getTabName() {
        return m_tabName;
    }

    public ShuffleboardTab getTab() {
        return Shuffleboard.getTab(m_tabName);
    }
}