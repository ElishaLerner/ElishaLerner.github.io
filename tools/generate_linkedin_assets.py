from pathlib import Path
import textwrap

from PIL import Image, ImageDraw, ImageFont, ImageOps


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "assets" / "linkedin"

COLORS = {
    "ink": "#16212c",
    "muted": "#556372",
    "paper": "#f5f7fa",
    "panel": "#ffffff",
    "line": "#d8e1ea",
    "blue": "#15365c",
    "cyan": "#0e8397",
    "orange": "#b85a2a",
    "dark": "#081017",
}


def font(name: str, size: int) -> ImageFont.FreeTypeFont:
    font_dir = Path("C:/Windows/Fonts")
    choices = {
        "bold": ["segoeuib.ttf", "arialbd.ttf"],
        "semi": ["seguisb.ttf", "segoeuib.ttf", "arialbd.ttf"],
        "regular": ["segoeui.ttf", "arial.ttf"],
    }[name]
    for choice in choices:
        path = font_dir / choice
        if path.exists():
            return ImageFont.truetype(str(path), size=size)
    return ImageFont.load_default()


FONTS = {
    "eyebrow": font("bold", 26),
    "banner_title": font("bold", 58),
    "banner_sub": font("regular", 29),
    "title": font("bold", 52),
    "subtitle": font("regular", 27),
    "small": font("semi", 22),
    "body": font("regular", 26),
}


def hex_to_rgb(value: str) -> tuple[int, int, int]:
    value = value.lstrip("#")
    return tuple(int(value[i : i + 2], 16) for i in (0, 2, 4))


def load_image(relative: str) -> Image.Image:
    return Image.open(ROOT / relative).convert("RGB")


def fit_cover(img: Image.Image, size: tuple[int, int], anchor: tuple[float, float] = (0.5, 0.5)) -> Image.Image:
    width, height = size
    scale = max(width / img.width, height / img.height)
    resized = img.resize((round(img.width * scale), round(img.height * scale)), Image.Resampling.LANCZOS)
    left = round((resized.width - width) * anchor[0])
    top = round((resized.height - height) * anchor[1])
    return resized.crop((left, top, left + width, top + height))


def fit_contain(img: Image.Image, size: tuple[int, int], bg: str = COLORS["paper"]) -> Image.Image:
    canvas = Image.new("RGB", size, bg)
    contained = ImageOps.contain(img, size, method=Image.Resampling.LANCZOS)
    x = (size[0] - contained.width) // 2
    y = (size[1] - contained.height) // 2
    canvas.paste(contained, (x, y))
    return canvas


def overlay_tint(img: Image.Image, color: str, alpha: int) -> Image.Image:
    layer = Image.new("RGBA", img.size, (*hex_to_rgb(color), alpha))
    return Image.alpha_composite(img.convert("RGBA"), layer).convert("RGB")


def text_width(draw: ImageDraw.ImageDraw, text: str, font_obj: ImageFont.ImageFont) -> int:
    return round(draw.textbbox((0, 0), text, font=font_obj)[2])


def wrap_text(draw: ImageDraw.ImageDraw, text: str, font_obj: ImageFont.ImageFont, max_width: int) -> list[str]:
    words = text.split()
    lines: list[str] = []
    current = ""
    for word in words:
        candidate = word if not current else f"{current} {word}"
        if text_width(draw, candidate, font_obj) <= max_width:
            current = candidate
        else:
            if current:
                lines.append(current)
            current = word
    if current:
        lines.append(current)
    return lines


def draw_wrapped(draw: ImageDraw.ImageDraw, xy: tuple[int, int], text: str, font_obj, fill: str, max_width: int, line_gap: int = 10) -> int:
    x, y = xy
    lines = wrap_text(draw, text, font_obj, max_width)
    line_height = draw.textbbox((0, 0), "Ag", font=font_obj)[3] + line_gap
    for line in lines:
        draw.text((x, y), line, fill=fill, font=font_obj)
        y += line_height
    return y


def rounded_rect(draw: ImageDraw.ImageDraw, box, radius: int, fill: str, outline: str | None = None, width: int = 1) -> None:
    draw.rounded_rectangle(box, radius=radius, fill=fill, outline=outline, width=width)


def make_banner() -> None:
    size = (1584, 396)
    canvas = Image.new("RGB", size, COLORS["blue"])
    draw = ImageDraw.Draw(canvas)

    collage_specs = [
        ("assets/hand/hardware/hardware-box-of-tea-poster.jpg", (0, 0, 330, 396), (0.47, 0.52)),
        ("assets/cad/printer/physical/m3id-retrofit-isometric-side.jpg", (330, 0, 610, 396), (0.50, 0.50)),
        ("assets/papers/amphibious_extra/swim-to-walk-overlay.png", (610, 0, 890, 396), (0.42, 0.50)),
    ]
    for rel, box, anchor in collage_specs:
        img = fit_cover(load_image(rel), (box[2] - box[0], box[3] - box[1]), anchor)
        img = overlay_tint(img, COLORS["dark"], 65)
        canvas.paste(img, box[:2])

    draw.rectangle((820, 0, 1584, 396), fill=COLORS["blue"])
    draw.rectangle((790, 0, 840, 396), fill=COLORS["blue"])
    draw.rectangle((820, 312, 1584, 396), fill=COLORS["cyan"])

    draw.text((890, 64), "ELISHA LERNER", fill="#d8eef4", font=FONTS["eyebrow"])
    draw.text((890, 108), "Robotics R&D", fill="white", font=FONTS["banner_title"])
    draw.text((890, 174), "Mechatronics | Additive Manufacturing | Controls", fill="#dce7ef", font=FONTS["banner_sub"])
    draw.text((890, 236), "Mechanism concept -> validated hardware", fill="#dce7ef", font=FONTS["banner_sub"])
    draw.text((890, 326), "PhD candidate | Available Summer 2026", fill="white", font=FONTS["small"])

    canvas.save(OUT / "linkedin-profile-banner.png", quality=95)


def make_card(filename: str, eyebrow: str, title: str, subtitle: str, image_rel: str, url_label: str, anchor=(0.5, 0.5), contain=False) -> None:
    size = (1200, 627)
    canvas = Image.new("RGB", size, COLORS["paper"])
    draw = ImageDraw.Draw(canvas)

    draw.rectangle((0, 0, 1200, 627), fill=COLORS["paper"])
    draw.rectangle((0, 0, 1200, 14), fill=COLORS["cyan"])
    draw.rectangle((0, 14, 12, 627), fill=COLORS["blue"])

    image_box = (635, 72, 1128, 485)
    source = load_image(image_rel)
    if contain:
        img = fit_contain(source, (image_box[2] - image_box[0], image_box[3] - image_box[1]), bg="#eef4f8")
    else:
        img = fit_cover(source, (image_box[2] - image_box[0], image_box[3] - image_box[1]), anchor=anchor)
    rounded_rect(draw, image_box, 8, COLORS["panel"], COLORS["line"])
    mask = Image.new("L", img.size, 0)
    ImageDraw.Draw(mask).rounded_rectangle((0, 0, img.width, img.height), radius=8, fill=255)
    canvas.paste(img, image_box[:2], mask)

    x = 72
    y = 80
    draw.text((x, y), eyebrow.upper(), fill=COLORS["cyan"], font=FONTS["eyebrow"])
    y += 48
    y = draw_wrapped(draw, (x, y), title, FONTS["title"], COLORS["blue"], 510, line_gap=8)
    y += 18
    y = draw_wrapped(draw, (x, y), subtitle, FONTS["subtitle"], COLORS["ink"], 525, line_gap=8)

    draw.rectangle((72, 545, 1128, 548), fill=COLORS["line"])
    draw.text((72, 568), url_label, fill=COLORS["muted"], font=FONTS["small"])
    canvas.save(OUT / filename, quality=95)


def make_square_slide(filename: str, eyebrow: str, title: str, bullets: list[str], image_rel: str, anchor=(0.5, 0.5), contain=False) -> Image.Image:
    size = (1080, 1080)
    canvas = Image.new("RGB", size, COLORS["paper"])
    draw = ImageDraw.Draw(canvas)
    draw.rectangle((0, 0, 1080, 18), fill=COLORS["cyan"])

    image_box = (92, 92, 988, 514)
    source = load_image(image_rel)
    img = fit_contain(source, (image_box[2] - image_box[0], image_box[3] - image_box[1]), bg="#eef4f8") if contain else fit_cover(source, (image_box[2] - image_box[0], image_box[3] - image_box[1]), anchor)
    rounded_rect(draw, image_box, 8, COLORS["panel"], COLORS["line"])
    mask = Image.new("L", img.size, 0)
    ImageDraw.Draw(mask).rounded_rectangle((0, 0, img.width, img.height), radius=8, fill=255)
    canvas.paste(img, image_box[:2], mask)

    draw.text((92, 575), eyebrow.upper(), fill=COLORS["cyan"], font=FONTS["eyebrow"])
    y = draw_wrapped(draw, (92, 625), title, FONTS["title"], COLORS["blue"], 880, line_gap=8)
    y += 20
    for bullet in bullets:
        draw.ellipse((96, y + 10, 110, y + 24), fill=COLORS["orange"])
        y = draw_wrapped(draw, (128, y), bullet, FONTS["body"], COLORS["ink"], 810, line_gap=8)
        y += 14
    draw.text((92, 1015), "elishalerner.github.io", fill=COLORS["muted"], font=FONTS["small"])
    canvas.save(OUT / filename, quality=95)
    return canvas


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    make_banner()

    cards = [
        (
            "featured-portfolio-overview.png",
            "Portfolio",
            "Robotic systems built from concept to validated hardware",
            "CAD, controls, firmware, fabrication, and experimental validation.",
            "assets/hand/system-architecture.png",
            "elishalerner.github.io",
            (0.5, 0.5),
            True,
        ),
        (
            "featured-robotic-hand.png",
            "Robotic hand",
            "Compliant FDM-compatible robotic hand",
            "Mechanism design, Simulink/MuJoCo workflow, Feetech servo control, current limits, and bench validation.",
            "assets/hand/hardware/hardware-box-of-tea-poster.jpg",
            "Portfolio case study",
            (0.45, 0.48),
            False,
        ),
        (
            "featured-controls-simulation.png",
            "Controls",
            "Simulink, MuJoCo, and robot-learning models",
            "PID tuning, TCP simulation boundary, RL coursework, and motion-tracking ML.",
            "assets/hand/controller-tuning-comparison.png",
            "Controls and simulation page",
            (0.5, 0.5),
            True,
        ),
        (
            "featured-printer-retrofit.png",
            "CAD + additive",
            "M3ID direct-drive printer retrofit",
            "Toolhead packaging, carriage adaptation, cooling duct design, physical install, and custom Klipper startup logic.",
            "assets/cad/printer/physical/m3id-retrofit-isometric-side.jpg",
            "CAD evidence page",
            (0.50, 0.50),
            False,
        ),
        (
            "featured-amphibious-robot.png",
            "System integration",
            "Shape-morphing amphibious robot",
            "Waterproof packaging, rotating morphing limbs, encoded motors, BLE routines, thermal actuation, and field testing.",
            "assets/papers/amphibious_extra/swim-to-walk-overlay.png",
            "Amphibious robot case study",
            (0.42, 0.50),
            False,
        ),
        (
            "featured-variable-stiffness-origami.png",
            "Variable stiffness",
            "Origami robots with active stiffness inserts",
            "Tendon-driven motion, PID heater loops, thermistor feedback, gait switching, locomotion, and adaptive grasping.",
            "assets/papers/modular_vsi/system-overview.png",
            "Origami systems case study",
            (0.5, 0.5),
            True,
        ),
        (
            "featured-motion-tracking-ml.png",
            "Robot learning",
            "Motion-tracking ML for adaptive origami",
            "Endpoint data, heater inputs, encoder actuation, TensorFlow/Keras models, and TFLite export.",
            "assets/coursework/ml-motion-capture-setup.png",
            "Controls coursework evidence",
            (0.50, 0.50),
            False,
        ),
    ]

    for args in cards:
        make_card(*args)

    slides = [
        make_square_slide(
            "carousel-01-overview.png",
            "Portfolio overview",
            "Robotics R&D from mechanism concept to validated hardware",
            ["PhD candidate finishing Summer 2026", "8+ years robotics and mechatronics R&D", "CAD, controls, firmware, fabrication, and validation"],
            "assets/hand/system-architecture.png",
            contain=True,
        ),
        make_square_slide(
            "carousel-02-hand.png",
            "Compliant robotic hand",
            "FDM-compatible hand with simulation-to-hardware controls",
            ["Three-motor compliant mechanism", "Simulink/MuJoCo control workflow", "Python Feetech servo tooling and bench demos"],
            "assets/hand/hardware/hardware-box-of-tea-poster.jpg",
            (0.45, 0.48),
        ),
        make_square_slide(
            "carousel-03-controls.png",
            "Controls and modeling",
            "Simulink, MuJoCo, and motion-control evidence",
            ["Plant/controller separation", "PID tuning and torque/current limit sweeps", "Robot-learning and dynamics coursework artifacts"],
            "assets/hand/controller-tuning-comparison.png",
            contain=True,
        ),
        make_square_slide(
            "carousel-04-printer.png",
            "Printer retrofit",
            "Practical machine integration for additive manufacturing",
            ["Right-extruder Smart Orbiter-style retrofit", "CAD packaging and cooling duct design", "Custom Klipper startup G-code"],
            "assets/cad/printer/physical/m3id-retrofit-isometric-side.jpg",
            (0.50, 0.50),
        ),
        make_square_slide(
            "carousel-05-amphibious.png",
            "Shape-morphing robot",
            "Amphibious system integration and validation",
            ["Waterproof body and morphing limbs", "Encoded motors, BLE modes, thermal actuation", "Tank tests and outdoor land-water transitions"],
            "assets/papers/amphibious_extra/swim-to-walk-overlay.png",
            (0.42, 0.50),
        ),
        make_square_slide(
            "carousel-06-origami.png",
            "Variable-stiffness robots",
            "Origami platforms with active stiffness control",
            ["SMP/PETG composites and active VSIs", "Thermistors, heaters, relay isolation, encoders", "Locomotion, gait switching, and adaptive grasping"],
            "assets/papers/modular_vsi/system-overview.png",
            contain=True,
        ),
        make_square_slide(
            "carousel-07-contact.png",
            "Explore the work",
            "Portfolio, source evidence, and publications",
            ["Full project pages: elishalerner.github.io", "Source files and control artifacts included", "Available for robotics R&D and mechatronics roles after PhD completion"],
            "assets/coursework/ml-motion-capture-setup.png",
            (0.50, 0.50),
        ),
    ]
    pdf_path = OUT / "linkedin-portfolio-carousel.pdf"
    slides[0].save(pdf_path, save_all=True, append_images=slides[1:], resolution=100.0)

    readme = OUT / "README-linkedin-assets.txt"
    readme.write_text(
        "\n".join(
            [
                "LinkedIn asset pack generated from the portfolio.",
                "",
                "Use linkedin-profile-banner.png as the profile cover image.",
                "Use featured-*.png as Featured-section link thumbnails, post images, or article cover starting points.",
                "Use linkedin-portfolio-carousel.pdf as a document/carousel post if you want a compact visual overview.",
                "",
                "Recommended dimensions used:",
                "- Profile cover: 1584 x 396",
                "- Featured/link cards: 1200 x 627",
                "- Carousel slides: 1080 x 1080, exported as a PDF",
            ]
        ),
        encoding="utf-8",
    )


if __name__ == "__main__":
    main()
