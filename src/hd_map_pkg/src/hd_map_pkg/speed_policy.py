"""Competition speed-policy annotations for the immutable KATRI HD map."""


def competition_speed_policy(dataset, config):
    """Validate and resolve the configured competition speed-exemption zone."""
    raw = (config or {}).get("competition_speed_policy")
    if not raw:
        return None

    default_limit = int(raw.get("default_limit_kph", 0))
    exemption = raw.get("exemption") or {}
    zone_id = str(exemption.get("id", "")).strip()
    start_link_id = str(exemption.get("start_link_id", "")).strip()
    end_link_id = str(exemption.get("end_link_id", "")).strip()
    road_ids = tuple(str(value).strip() for value in
                     exemption.get("road_ids", []) if str(value).strip())

    if default_limit <= 0:
        raise ValueError("competition default_limit_kph must be positive")
    if not zone_id or not start_link_id or not end_link_id or not road_ids:
        raise ValueError(
            "competition speed exemption requires id, boundary links and road_ids")
    for link_id in (start_link_id, end_link_id):
        if link_id not in dataset.links:
            raise ValueError(
                "competition speed exemption link is absent: {}".format(link_id))

    selected_links = sorted(
        link_id for link_id, link in dataset.links.items()
        if str(link.get("road_id", "")) in road_ids)
    if not selected_links:
        raise ValueError("competition speed exemption road_ids select no links")
    for link_id in (start_link_id, end_link_id):
        if link_id not in selected_links:
            raise ValueError(
                "competition speed exemption boundary is outside configured roads: {}"
                .format(link_id))

    start_link = dataset.links[start_link_id]
    end_link = dataset.links[end_link_id]
    if not start_link.get("points") or not end_link.get("points"):
        raise ValueError("competition speed exemption boundary has no geometry")

    return {
        "default_limit_kph": default_limit,
        "id": zone_id,
        "label": str(exemption.get("label", zone_id)),
        "source": str(exemption.get("source", "")),
        "start_link_id": start_link_id,
        "end_link_id": end_link_id,
        "road_ids": road_ids,
        "link_ids": tuple(selected_links),
        "start_point": start_link["points"][0],
        "end_point": end_link["points"][-1],
    }


def competition_speed_tags(link_id, policy):
    """Return explicit mission-policy tags without replacing source speed_limit."""
    if policy is None:
        return {}
    exempt = str(link_id) in policy["link_ids"]
    tags = {
        "molit:competition_speed_limit_kph": str(policy["default_limit_kph"]),
        "molit:competition_speed_limit_exempt": "yes" if exempt else "no",
    }
    if exempt:
        tags["molit:competition_speed_zone"] = policy["id"]
    return tags
