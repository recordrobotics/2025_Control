package frc.robot.dashboard;

import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;
import java.util.function.Supplier;

public abstract class AbstractLayout {
  protected List<SendableBuilder> sendableBuilders = new ArrayList<SendableBuilder>();
  protected List<IValueSendable> valueSendables = new ArrayList<IValueSendable>();

  public interface IValueSendable {
    Boolean update();
  }

  public static class ValueSendable<T> implements IValueSendable {
    private Function<T, Boolean> consumer;
    private Supplier<T> supplier;

    public ValueSendable(Function<T, Boolean> consumer, Supplier<T> supplier) {
      this.consumer = consumer;
      this.supplier = supplier;
    }

    public Boolean update() {
      return consumer.apply(supplier.get());
    }
  }

  public abstract void switchTo();

  protected abstract NetworkTable getNetworkTable();

  public SendableBuilder buildSendable(String topic, NTSendable sendable) {
    var m_builder = new SendableBuilderImpl();
    m_builder.setTable(getNetworkTable().getSubTable(topic));
    sendable.initSendable(m_builder);
    m_builder.startListeners();
    m_builder.update();
    sendableBuilders.add(m_builder);
    return m_builder;
  }

  public <T> void addValueSendable(String topic, Supplier<T> supplier, String type) {
    GenericPublisher pub = getNetworkTable().getTopic(topic).genericPublish(type);
    valueSendables.add(new ValueSendable<T>(pub::setValue, supplier));
  }

  public void update() {
    sendableBuilders.forEach(SendableBuilder::update);
    valueSendables.forEach(IValueSendable::update);
  }
}
