#!/usr/bin/env python3


from robot_ai_interfaces.msg import GuestAppearance


def describe_person(profile):
    parts = []
    if profile.name:
        parts.append(profile.name)
    else:
        parts.append("this guest")

    if profile.favorite_drink:
        parts.append(f"whose favourite drink is {profile.favorite_drink}")

    appearance_parts = describe_appearance(profile.appearance)
    if appearance_parts:
        parts.append(f"and who {appearance_parts}")

    return " ".join(parts)


def _flag_text(value, yes_text):
    if value == GuestAppearance.YES:
        return yes_text
    return ""


def describe_appearance(appearance):
    parts = []
    if appearance.shirt_color:
        parts.append(f"is wearing a {appearance.shirt_color} shirt")
    glasses_text = _flag_text(appearance.glasses, "wears glasses")
    if glasses_text:
        parts.append(glasses_text)
    mask_text = _flag_text(appearance.face_mask, "is wearing a face mask")
    if mask_text:
        parts.append(mask_text)
    hat_text = _flag_text(appearance.hat, "is wearing a hat")
    if hat_text:
        parts.append(hat_text)
    return ", ".join(parts)


def build_introduction_lines(source_profile, target_profiles, include_description_bonus=False):
    lines = []
    source_desc = describe_person(source_profile)

    for target in target_profiles:
        target_name = target.name if target.name else target.role
        lines.append(f"{target_name}, this is {source_desc}.")

    if include_description_bonus and target_profiles:
        first_known = target_profiles[-1]
        known_name = first_known.name if first_known.name else "the other guest"
        lines.append(f"For your information, {known_name} is already seated here.")

    return lines
