package frc.robot.utilities;

public class TunablePIDF {
  private final TunableDouble m_p, m_i, m_d, m_ff;
  private PIDF m_pidf;

  public TunablePIDF(String key, PIDF defaultValue) {
    m_p = new TunableDouble(key + ".P", defaultValue.p());
    m_i = new TunableDouble(key + ".I", defaultValue.i());
    m_d = new TunableDouble(key + ".D", defaultValue.d());
    m_ff = new TunableDouble(key + ".FF", defaultValue.ff());
  }

  public PIDF get() {
    update();
    return m_pidf;
  }

  private void update() {
    double p = m_p.get();
    double i = m_i.get();
    double d = m_d.get();
    double ff = m_ff.get();
    m_pidf = new PIDF(p, i, d, ff);
  }

  public boolean hasChanged() {
    return m_p.hasChanged() || m_i.hasChanged() || m_d.hasChanged() || m_ff.hasChanged();
  }
}