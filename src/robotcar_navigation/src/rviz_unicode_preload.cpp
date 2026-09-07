// RViz 2 Jazzy's MovableText currently treats Ogre::String UTF-8 data as one
// glyph per byte.  This library is preloaded only into the RViz process and
// interposes the affected methods with Unicode-aware implementations.  It
// also substitutes a system CJK font for RViz's Latin-only default font.

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <dlfcn.h>

#include <OgreHardwareBufferManager.h>
#include <OgreMaterialManager.h>
#include <OgreVector.h>
#include <Overlay/OgreFont.h>
#include <Overlay/OgreFontManager.h>

// Access is needed because these are the exact methods being ABI-interposed.
// No object layout is changed; the header comes from the running RViz build.
#define private public
#include "rviz_rendering/objects/movable_text.hpp"
#undef private

namespace
{

constexpr char kMaterialGroup[] = "rviz_rendering";
constexpr char kCjkFontName[] = "ANTBot CJK";
constexpr float kEffectiveCharacterHeightFactor = 2.0F;

std::vector<std::uint32_t> decodeUtf8(const std::string & input)
{
  std::vector<std::uint32_t> result;
  result.reserve(input.size());

  std::size_t index = 0;
  while (index < input.size()) {
    const auto first = static_cast<unsigned char>(input[index]);
    std::uint32_t codepoint = 0;
    std::size_t length = 0;

    if (first < 0x80U) {
      codepoint = first;
      length = 1;
    } else if ((first & 0xE0U) == 0xC0U) {
      codepoint = first & 0x1FU;
      length = 2;
    } else if ((first & 0xF0U) == 0xE0U) {
      codepoint = first & 0x0FU;
      length = 3;
    } else if ((first & 0xF8U) == 0xF0U) {
      codepoint = first & 0x07U;
      length = 4;
    } else {
      result.push_back(0xFFFDU);
      ++index;
      continue;
    }

    if (index + length > input.size()) {
      result.push_back(0xFFFDU);
      break;
    }

    bool valid = true;
    for (std::size_t offset = 1; offset < length; ++offset) {
      const auto next = static_cast<unsigned char>(input[index + offset]);
      if ((next & 0xC0U) != 0x80U) {
        valid = false;
        break;
      }
      codepoint = (codepoint << 6U) | (next & 0x3FU);
    }

    const bool overlong =
      (length == 2 && codepoint < 0x80U) ||
      (length == 3 && codepoint < 0x800U) ||
      (length == 4 && codepoint < 0x10000U);
    if (!valid || overlong || codepoint > 0x10FFFFU ||
      (codepoint >= 0xD800U && codepoint <= 0xDFFFU))
    {
      result.push_back(0xFFFDU);
      ++index;
      continue;
    }

    result.push_back(codepoint);
    index += length;
  }
  return result;
}

std::filesystem::path findCjkFont()
{
  if (const char * configured = std::getenv("ANTBOT_RVIZ_CJK_FONT")) {
    const std::filesystem::path path(configured);
    if (std::filesystem::is_regular_file(path)) {
      return path;
    }
  }

  // Noto CJK includes Latin, Greek, Cyrillic, CJK and the common symbol
  // blocks.  DroidSansFallback is deliberately last: despite its name, the
  // Ubuntu build has CJK glyphs but no printable ASCII except U+0020.
  constexpr std::array<const char *, 3> candidates = {
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf",
    "/usr/share/fonts/truetype/arphic/uming.ttc"};
  for (const char * candidate : candidates) {
    if (std::filesystem::is_regular_file(candidate)) {
      return candidate;
    }
  }
  return {};
}

const char * cjkFontName()
{
  static bool initialized = false;
  static bool available = false;
  if (initialized) {
    return available ? kCjkFontName : "Liberation Sans";
  }
  initialized = true;

  const std::filesystem::path path = findCjkFont();
  if (path.empty()) {
    return "Liberation Sans";
  }

  auto & resource_manager = Ogre::ResourceGroupManager::getSingleton();
  resource_manager.addResourceLocation(
    path.parent_path().string(), "FileSystem", kMaterialGroup);

  auto & font_manager = Ogre::FontManager::getSingleton();
  Ogre::FontPtr font = font_manager.getByName(kCjkFontName, kMaterialGroup);
  if (!font) {
    font = font_manager.create(kCjkFontName, kMaterialGroup);
    font->setType(Ogre::FT_TRUETYPE);
    font->setSource(path.filename().string());
    font->setTrueTypeSize(18.0F);
    font->setTrueTypeResolution(96U);
    font->clearCodePointRanges();
    // Mixed Chinese/English labels plus common European scripts.
    font->addCodePointRange({0x0020U, 0x024FU});
    font->addCodePointRange({0x0370U, 0x052FU});
    font->addCodePointRange({0x1E00U, 0x1EFFU});
    // Punctuation, currency, letter-like, arrows, maths, technical,
    // geometric, dingbats and supplemental arrows/symbols.
    font->addCodePointRange({0x2000U, 0x2BFFU});
    // CJK punctuation, kana/bopomofo and compatibility characters.
    font->addCodePointRange({0x3000U, 0x303FU});
    font->addCodePointRange({0x3040U, 0x33FFU});
    font->addCodePointRange({0x3400U, 0x4DBFU});
    font->addCodePointRange({0x4E00U, 0x9FFFU});
    font->addCodePointRange({0xF900U, 0xFAFFU});
    font->addCodePointRange({0xFE10U, 0xFE6FU});
    font->addCodePointRange({0xFF00U, 0xFFEFU});
  }
  available = true;
  return kCjkFontName;
}

struct TextBuffer
{
  float * buffer;
  Ogre::Vector3 minimum{std::numeric_limits<float>::max()};
  Ogre::Vector3 maximum{std::numeric_limits<float>::lowest()};
  Ogre::Real maximum_squared_radius{0.0F};
  float top{0.0F};
  float left{0.0F};
  Ogre::Font::UVRect texture_coordinates;

  explicit TextBuffer(float * target)
  : buffer(target) {}

  void addPosition(float plus_left, float minus_top)
  {
    const Ogre::Vector3 position(left + plus_left, top - minus_top, 0.0F);
    *buffer++ = position.x;
    *buffer++ = position.y;
    *buffer++ = position.z;
    minimum.makeFloor(position);
    maximum.makeCeil(position);
    maximum_squared_radius = std::max(maximum_squared_radius, position.squaredLength());
  }

  void addTexture(float x, float y)
  {
    *buffer++ = x;
    *buffer++ = y;
  }

  void addTopLeft()
  {
    addPosition(0.0F, 0.0F);
    addTexture(texture_coordinates.left, texture_coordinates.top);
  }

  void addBottomLeft(float height)
  {
    addPosition(0.0F, 2.0F * height);
    addTexture(texture_coordinates.left, texture_coordinates.bottom);
  }

  void addTopRight(float width)
  {
    addPosition(2.0F * width, 0.0F);
    addTexture(texture_coordinates.right, texture_coordinates.top);
  }

  void addBottomRight(float width, float height)
  {
    addPosition(2.0F * width, 2.0F * height);
    addTexture(texture_coordinates.right, texture_coordinates.bottom);
  }
};

}  // namespace

namespace rviz_rendering
{

void MovableText::setFontName(const Ogre::String & font_name)
{
  using Original = void (*)(MovableText *, const Ogre::String &);
  static Original original = reinterpret_cast<Original>(
    dlsym(
      RTLD_NEXT,
      "_ZN14rviz_rendering11MovableText11setFontNameERKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEE"));
  if (!original) {
    throw std::runtime_error("Unable to locate RViz MovableText::setFontName");
  }
  original(this, font_name == "Liberation Sans" ? cjkFontName() : font_name.c_str());
}

unsigned int MovableText::calculateVertexCount() const
{
  unsigned int count = 0;
  for (const std::uint32_t character : decodeUtf8(caption_)) {
    if (character != ' ' && character != '\n') {
      count += 6U;
    }
  }
  return count;
}

void MovableText::calculateTotalDimensionsForPositioning(
  float & total_height, float & total_width) const
{
  const Ogre::Real effective_height =
    char_height_ * kEffectiveCharacterHeightFactor;
  total_height = effective_height;
  total_width = 0.0F;
  float current_width = 0.0F;

  for (const std::uint32_t character : decodeUtf8(caption_)) {
    if (character == '\n') {
      total_height += effective_height + line_spacing_;
      total_width = std::max(total_width, current_width);
      current_width = 0.0F;
    } else if (character == ' ') {
      current_width += space_width_;
    } else {
      current_width += font_->getGlyphAspectRatio(character) * effective_height;
    }
  }
  total_width = std::max(total_width, current_width);
}

void MovableText::fillVertexBuffer(
  Ogre::HardwareVertexBufferSharedPtr & position_and_texture_buffer,
  float top, float starting_left)
{
  const Ogre::Real effective_height =
    char_height_ * kEffectiveCharacterHeightFactor;
  auto * hardware_buffer = static_cast<float *>(
    position_and_texture_buffer->lock(Ogre::HardwareBuffer::HBL_DISCARD));
  TextBuffer buffer(hardware_buffer);
  buffer.left = starting_left;
  buffer.top = top;

  for (const std::uint32_t character : decodeUtf8(caption_)) {
    if (character == '\n') {
      buffer.left = starting_left;
      buffer.top -= effective_height + line_spacing_;
      continue;
    }
    if (character == ' ') {
      buffer.left += space_width_;
      continue;
    }

    const Ogre::Real aspect_ratio = font_->getGlyphAspectRatio(character);
    buffer.texture_coordinates = font_->getGlyphTexCoords(character);
    const float character_width = aspect_ratio * char_height_;
    buffer.addTopLeft();
    buffer.addBottomLeft(char_height_);
    buffer.addTopRight(character_width);
    buffer.addTopRight(character_width);
    buffer.addBottomLeft(char_height_);
    buffer.addBottomRight(character_width, char_height_);
    buffer.left += aspect_ratio * effective_height;
  }

  position_and_texture_buffer->unlock();
  mBox = Ogre::AxisAlignedBox(buffer.minimum, buffer.maximum);
  radius_ = Ogre::Math::Sqrt(buffer.maximum_squared_radius);
}

}  // namespace rviz_rendering
