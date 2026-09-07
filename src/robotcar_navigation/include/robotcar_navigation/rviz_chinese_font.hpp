#ifndef ROBOTCAR_NAVIGATION__RVIZ_CHINESE_FONT_HPP_
#define ROBOTCAR_NAVIGATION__RVIZ_CHINESE_FONT_HPP_

#include <QApplication>
#include <QFont>
#include <QFontDatabase>
#include <QStringList>

namespace robotcar_navigation
{
namespace rviz_plugins
{

inline void configureChineseUiFont()
{
  static bool configured = false;
  if (configured) {
    return;
  }
  configured = true;

  const QStringList available = QFontDatabase().families();
  const QStringList preferred = {
    QStringLiteral("Noto Sans CJK SC"),
    QStringLiteral("Droid Sans Fallback"),
    QStringLiteral("Noto Sans CJK TC")};

  for (const QString & family : preferred) {
    if (available.contains(family)) {
      QFont font = QApplication::font();
      font.setFamily(family);
      QApplication::setFont(font);
      return;
    }
  }
}

}  // namespace rviz_plugins
}  // namespace robotcar_navigation

#endif  // ROBOTCAR_NAVIGATION__RVIZ_CHINESE_FONT_HPP_
