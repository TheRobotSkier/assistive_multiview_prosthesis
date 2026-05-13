#!/usr/bin/env python3
"""
Generate printable OpenCV ArUco markers directly as PDF.

Default:
- Dictionary: DICT_6X6_1000
- Marker IDs: 0, 1, 2, 3
- Output: one multi-page A4 PDF
- One marker per A4 page
- Marker size: 160 mm square

Print the PDF at:
- 100% / Actual Size
- A4 paper
- No "Fit to page" scaling
"""

import argparse
import io
import sys
from pathlib import Path

try:
    import cv2
    import numpy as np
except ImportError as exc:
    print("ERROR: This script needs OpenCV Python and NumPy.", file=sys.stderr)
    raise exc

try:
    from PIL import Image
except ImportError as exc:
    print("ERROR: This script needs Pillow.", file=sys.stderr)
    raise exc

try:
    from reportlab.lib.pagesizes import A4
    from reportlab.lib.units import mm
    from reportlab.lib.utils import ImageReader
    from reportlab.pdfgen import canvas
except ImportError as exc:
    print("ERROR: This script needs reportlab.", file=sys.stderr)
    raise exc


def get_aruco_dictionary():
    if not hasattr(cv2, "aruco"):
        raise RuntimeError(
            "cv2.aruco is not available. Your OpenCV Python was built without aruco/contrib support."
        )
    return cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_1000)


def generate_marker(dictionary, marker_id: int, marker_px: int) -> np.ndarray:
    marker = np.zeros((marker_px, marker_px), dtype=np.uint8)

    # OpenCV version compatibility
    if hasattr(cv2.aruco, "generateImageMarker"):
        cv2.aruco.generateImageMarker(dictionary, marker_id, marker_px, marker, 1)
    elif hasattr(cv2.aruco, "drawMarker"):
        cv2.aruco.drawMarker(dictionary, marker_id, marker_px, marker, 1)
    else:
        raise RuntimeError(
            "No compatible ArUco generation function found in cv2.aruco."
        )

    return marker


def marker_to_image_reader(marker: np.ndarray):
    pil_img = Image.fromarray(marker, mode="L")
    buf = io.BytesIO()
    pil_img.save(buf, format="PNG")
    buf.seek(0)
    return ImageReader(buf)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--ids",
        nargs="+",
        type=int,
        default=[0, 1, 2, 3],
        help="ArUco IDs to generate. Default: 0 1 2 3",
    )
    parser.add_argument(
        "--outdir",
        default="generated_aruco_a4",
        help="Output directory. Default: generated_aruco_a4",
    )
    parser.add_argument(
        "--filename",
        default=None,
        help="Output PDF filename. Default is auto-generated.",
    )
    parser.add_argument(
        "--dpi",
        type=int,
        default=600,
        help="Marker rasterization DPI before embedding in PDF. Default: 600",
    )
    parser.add_argument(
        "--marker-mm",
        type=float,
        default=104.16667,
        help="Marker side length in millimeters. Default: 104.16667",
    )
    parser.add_argument(
        "--top-margin-mm",
        type=float,
        default=35.0,
        help="Top margin before the marker. Default: 35",
    )
    parser.add_argument(
        "--with-label",
        action="store_true",
        help="Add a label below each marker.",
    )
    args = parser.parse_args()

    for marker_id in args.ids:
        if marker_id < 0 or marker_id >= 1000:
            raise ValueError("DICT_6X6_1000 marker IDs must be in range 0..999")

    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)

    if args.filename is None:
        ids_str = "_".join(str(i) for i in args.ids)
        pdf_name = f"aruco_DICT_6X6_1000_ids_{ids_str}_A4_{int(args.marker_mm)}mm.pdf"
    else:
        pdf_name = args.filename

    pdf_path = outdir / pdf_name

    dictionary = get_aruco_dictionary()

    # Raster size used internally before embedding into vector PDF page layout
    marker_px = round(args.marker_mm / 25.4 * args.dpi)

    page_w_pt, page_h_pt = A4
    marker_size_pt = args.marker_mm * mm
    top_margin_pt = args.top_margin_mm * mm
    print(f"PDF page size: {page_w_pt / mm:.1f} mm x {page_h_pt / mm:.1f} mm")
    print(f"Marker size: {marker_size_pt / mm:.1f} mm")
    print(f"Top margin: {top_margin_pt / mm:.1f} mm")

    if marker_size_pt > page_w_pt * 0.95:
        raise ValueError("Marker too large for A4 width. Reduce --marker-mm.")

    c = canvas.Canvas(str(pdf_path), pagesize=A4)

    print(f"Generating PDF: {pdf_path}")
    print(f"Marker IDs: {args.ids}")
    print(f"Marker size: {args.marker_mm:.1f} mm")
    print(f"Internal rasterization: {args.dpi} dpi")
    print("One A4 page per marker")

    for marker_id in args.ids:
        marker = generate_marker(dictionary, marker_id, marker_px)
        img_reader = marker_to_image_reader(marker)

        x_pt = (page_w_pt - marker_size_pt) / 2.0
        y_top_pt = page_h_pt - top_margin_pt
        y_pt = y_top_pt - marker_size_pt

        c.setFont("Helvetica", 11)
        c.drawImage(
            img_reader,
            x_pt,
            y_pt,
            width=marker_size_pt,
            height=marker_size_pt,
            preserveAspectRatio=True,
            mask="auto",
        )

        if args.with_label:
            label = f"OpenCV ArUco DICT_6X6_1000   ID {marker_id}   size {args.marker_mm:.1f} mm"
            text_y = y_pt - 12 * mm
            c.setFont("Helvetica", 10)
            c.drawCentredString(page_w_pt / 2.0, text_y, label)

        c.showPage()

    c.save()

    print(f"Wrote: {pdf_path.resolve()}")
    print()
    print("Print instructions:")
    print("- Print on A4 paper")
    print("- Choose 'Actual Size' or '100%'")
    print("- Disable 'Fit to page' or scaling")
    print("- Keep the white border around the marker")


if __name__ == "__main__":
    main()
